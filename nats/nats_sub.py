import asyncio
import nats
import json
import time
import re
from drone_thing import *
from pymavlink import mavutil

telem_task = None # This ensures that there's only ever one instance of the telemetry stream being ran so that there aren't conflicts

async def send_telem_stream(drone, nc, lock):
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
            await nc.publish("drone.telem.stream", json.dumps(telem_payload).encode())

        await asyncio.sleep(0.5)

async def process_command(drone, nc, cmd, lock, movement_lock):
    global telem_task

    if cmd is None:
        return{"status": "error","message": "Missing command"}

    cmd = cmd.strip().lower()

    if cmd == "telem":
        if telem_task is None or telem_task.done():
            print("Starting telemetry stream...")
            telem_task = asyncio.create_task(send_telem_stream(drone, nc, lock))
            return {"status": "success", "message": "Stream started"}
        else:
            print("Telemetry stream already active.")
            return {"status": "ignored", "message": "Stream already running"}

   elif cmd.startswith("move"):
    move_match = re.fullmatch(r"move\s+(forward|back|left|right)\s+(\d+\.?\d*)", cmd)
    if not move_match:
        return {"status": "error", "message": "Use 'move forward 5', 'move back 5', 'move left 5', or 'move right 5'."}
    direction = move_match.group(1)
    distance = float(move_match.group(2))
    if distance <= 0:
        return {"status": "error", "message": "Movement distance must be positive."}
    print(f"Command to move {direction} by {distance} meters understood")
    async with movement_lock:
        async with lock:
            clear_all_overrides(drone)
            guided = await set_mode(drone, "GUIDED")
        if not guided:
            return {"status": "error", "executed": cmd, "message": "GUIDED mode was not confirmed."}
        worked = await move_rel(drone, direction, distance, lock)
        loitered = await hold_pos(drone, lock)
    if worked and loitered:
        return {"status": "success", "executed": cmd, "message": "Movement completed; drone is now loitering."}
    return {"status": "error", "executed": cmd, "message": "Movement failed or timed out; LOITER was requested."}

    elif cmd == "fly up": # Basic throttle test only
    print("Command to fly up understood. Running basic throttle test.")
    async with movement_lock:
        async with lock:
            clear_all_overrides(drone)
            stabilized = await set_mode(drone, "STABILIZE")
        if not stabilized:
            return {"status": "error", "executed": cmd, "message": "STABILIZE mode was not confirmed."}
        async with lock:
            armed = await arm_vehicle(drone)
        if not armed:
            async with lock:
                clear_all_overrides(drone)
            return {"status": "error", "executed": cmd, "message": "Vehicle failed to arm."}
        try:
            await movement(drone, duration=1.0, lock=lock, throttle_val=1550)
            worked = True
        except Exception as error:
            print(f"Fly-up test error: {error}")
            worked = False
        finally:
            async with lock:
                clear_all_overrides(drone)
                await disarm_vehicle(drone)

    if worked:
        return {"status": "success", "executed": cmd, "message": "Basic throttle test completed."}

    return {"status": "error", "executed": cmd, "message": "Basic throttle test failed."}

    elif cmd == "takeoff":
    return {
        "status": "error",
        "executed": cmd,
        "message": "Takeoff is temporarily disabled until the dedicated MAVLink takeoff helper is added."
    }
            
    
   elif cmd == "disarm":
    print("Explicit disarm command received.")
    async with lock:
        disarmed = await disarm_vehicle(drone)
        clear_all_overrides(drone)
    if disarmed:
        return {"status": "success", "executed": cmd}
    return {"status": "error", "executed": cmd, "message": "Vehicle did not confirm disarming."}

    elif cmd in ("stop telem","stop telemetry"):
        print("Command to stop telemetry understood. Stopping telemetry")
        if telem_task is not None and not telem_task.done():
            telem_task.cancel()
            telem_task = None
        return{"status": "success", "executed": cmd}

    elif cmd == "clear overrides":
        print("Command to clear overrides understood. Clearing overrides")
        async with lock:
            clear_all_overrides(drone)
        return {"status": "success", "executed": cmd}
    
elif cmd.startswith("set height"):
    height_match = re.fullmatch(r"set height\s+(up|down|increase|decrease)\s+(\d+\.?\d*)", cmd)
    if not height_match:
        return {"status": "error", "message": "Use 'set height up 1.0' or 'set height down 0.5'."}
    direction = height_match.group(1)
    distance = float(height_match.group(2))
    if distance <= 0:
        return {"status": "error", "message": "Height change must be positive."}
    async with movement_lock:
        async with lock:
            clear_all_overrides(drone)
            guided = await set_mode(drone, "GUIDED")
        if not guided:
            return {"status": "error", "executed": cmd, "message": "GUIDED mode was not confirmed."}
        if direction in ("up", "increase"):
            print(f"Command to increase height by {distance} meters understood")
            worked = await increase_height(drone, distance, lock)
        else:
            print(f"Command to decrease height by {distance} meters understood")
            worked = await decrease_height(drone, distance, lock)
        loitered = await hold_pos(drone, lock)
    if worked and loitered:
        return {"status": "success", "executed": cmd, "message": "Height changed; drone is now loitering."}
    return {"status": "error", "executed": cmd, "message": "Height movement failed or timed out; LOITER was requested."}
        
   elif cmd.startswith("throttle"):
    return {
        "status": "error",
        "executed": cmd,
        "message": "Raw throttle control is disabled in the production controller."
    }

async def main():
    # Connect to a NATS server
    nc = await nats.connect("nats://localhost:4222")

    PORT = "/dev/ttyS4"
    BAUD = 57600
    drone = mavutil.mavlink_connection(PORT, BAUD)
    drone.source_system = 255
    drone_lock = asyncio.Lock()
    movement_lock = asyncio.Lock()

    async with drone_lock:
        initialized = await initialize_telem(drone)

    if not initialized:
        print("MAVLink telemetry initialization failed")
        await nc.close()
        return

    print("MAVLink initialized successfully")

    async def message_handler(msg):
        try:
            request = json.loads(msg.data.decode())
            action = request.get("action")
        except json.JSONDecodeError:
            print("Received invalid JSON. Ignoring.")
            return

        response_payload = await process_command(drone, nc, action, drone_lock, movement_lock)
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
