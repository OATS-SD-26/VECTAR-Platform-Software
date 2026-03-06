import asyncio
import nats
from drone_thing import *

async def send_telem_stream(nc, msg):
    drone = initialize_telem()
    while True:
        t = get_telem(drone)
        response = f"Roll: {t["roll"]}, Pitch: {t["pitch"]}, Yaw: {t["yaw"]}"
        await nc.publish("drone.telem.stream", response.encode())
        await asyncio.sleep(0.5)

async def process_command(data):
    if data == "fly forward":
        print("Command to fly forward understood. Flying forward.")
    elif data == "fly up":
        print("Command to fly up understood. Flying up.")

async def main():
    # Connect to a NATS server
    nc = await nats.connect("nats://localhost:4222")
    commands = ["fly forward", "fly up"]
    telem_task = None

    async def message_handler(msg):
        data = msg.data.decode()
        if data == "telem":
            if telem_task is None or telem_task.done():
                print("Starting telemetry stream...")
                asyncio.create_task(send_telem_stream(nc, msg))
            else:
                print("Telemetry stream already active.")
        else:
            response = True if data in commands else False
            if response: await process_command(data)
            await msg.respond(str(response).encode())

    sub = await nc.subscribe("drone", cb=message_handler)
    print(f"Subscribed to 'drone', waiting for messages...")

    # Keep the connection alive to receive messages
    try:
        await asyncio.Future() # Run forever
    except KeyboardInterrupt:
        pass
    finally:
        # Drain messages and close the connection
        await sub.unsubscribe()
        await nc.drain()
        await nc.close()

if __name__ == '__main__':
    asyncio.run(main())