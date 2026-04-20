import asyncio
import nats
import json
import time
import re
from drone_thing import *

telem_task = None # This ensures that there's only ever one instance of the telemetry stream being ran so that there aren't conflicts

async def send_telem_stream(drone, nc, lock):
	async with lock:
		await initialize_telem(drone)

	while True:
		async with lock:
			t = await get_telem(drone)
			
		if t is not None:
			telem_payload = {
				"timestamp": time.time(),
				"device": "drone",
				"data": {
					"roll": t["roll"],
					"pitch": t["pitch"],
					"yaw": t["yaw"],
					"lat": t["lat"],
					"lon": t["lon"],
					"alt": t["alt"],
					"hdg": t["hdg"]
				}
			}

			json_msg = json.dumps(telem_payload).encode()
			await nc.publish("drone.telem.stream", json_msg)
		await asyncio.sleep(0.5)

async def process_command(drone, nc, cmd, lock):
	global telem_task

	if cmd == "telem":
		if telem_task is None or telem_task.done():
			print("Starting telemetry stream...")
			telem_task = asyncio.create_task(send_telem_stream(drone, nc, lock))
			return {"status": "success", "message": "Stream started"}
		else:
			print("Telemetry stream already active.")
			return {"status": "ignored", "message": "Stream already running"}
		
	elif cmd == "fly forward":
		print("Command to fly forward understood. Flying forward.")
		return {"status": "success", "executed": cmd}
	
	elif cmd == "fly up":
		print("Command to fly up understood. Flying up.")
		return {"status": "success", "executed": cmd}
	
	elif cmd.startswith("throttle"):
		throttle_match = re.fullmatch(r"throttle\s+(\d+),\s*(\d+\.?\d*)", cmd)
		if throttle_match:
			pwm = int(throttle_match.group(1))
			duration = float(throttle_match.group(2))
			if not (1000 <= pwm <= 2000) or not (duration > 0):
				return{"status": "error", "message": "Invalid throttle command. Must be entered in the form \'throttle x, y\', where \'x\' is the pwm value (an int between 1000 and 2000) and \'y\' is the duration (an int or float greater than 0)"}
			async with lock:
				set_mode(drone, "STABILIZE")
				await arm_vehicle(drone)
			await throttle_continuous(drone, pwm, duration, lock)
			async with lock:
				await disarm_vehicle(drone)
			return {"status": "success", "executed": cmd}
		else:
			return{"status": "error", "message": "Invalid throttle command. Must be entered in the form \'throttle x, y\', where \'x\' is the pwm value (an int between 1000 and 2000) and \'y\' is the duration (an int or float greater than 0)"}
		
	else:
		return {"status": "error", "message": "Unknown command"}


async def main():
	# Connect to a NATS server
	nc = await nats.connect("nats://localhost:4222")

	PORT = "/dev/ttyS4"
	BAUD = 57600
	drone = mavutil.mavlink_connection(PORT, BAUD)
	drone.source_system = 255
	drone_lock = asyncio.Lock()

	async def message_handler(msg):
		try:
			request = json.loads(msg.data.decode())
			action = request.get("action")
		except json.JSONDecodeError:
			print("Received invalid JSON. Ignoring.")
			return

		response_payload = await process_command(drone, nc, action, drone_lock)
		if msg.reply:
			await msg.respond(json.dumps(response_payload).encode())

	sub = await nc.subscribe("drone", cb=message_handler)
	print(f"Subscribed to 'drone', waiting for messages...")

	# Keep the connection alive to receive messages
	try:
		await asyncio.Future() # Run forever
	except KeyboardInterrupt:
		pass
	finally:
		global telem_task
		if telem_task and not telem_task.done():
			telem_task.cancel()
		# Drain messages and close the connection
		await sub.unsubscribe()
		await nc.drain()
		await nc.close()

if __name__ == '__main__':
	asyncio.run(main())