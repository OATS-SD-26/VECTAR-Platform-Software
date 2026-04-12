import sys
import asyncio
import nats
import json
from nats.errors import TimeoutError, NoRespondersError

async def send_command(nc, command):
    payload = {"action": command}
    command_msg = json.dumps(payload).encode()
    response = await nc.request("drone", command_msg, timeout=20)
    response_data = json.loads(response.data.decode())
    print(f"Command sent: {command}")
    print(f"Drone Response: {json.dumps(response_data, indent=2)}")

async def request_telem(nc):
    sub = await nc.subscribe("drone.telem.stream")
    request_msg = json.dumps({"action":"telem"}).encode()
    await nc.publish("drone", request_msg)
    print("Waiting for telemetry stream...")

    async for msg in sub.messages:
        try:
            telem_data = json.loads(msg.data.decode())
            t = telem_data.get("data", {}) # Second argument is a default value if "data" key isn't found
            timestamp = float(telem_data.get('timestamp', 0.0))
            roll = float(t.get('roll', 0.0))
            pitch = float(t.get('pitch', 0.0))
            yaw = float(t.get('yaw', 0.0))

            print(f"[{timestamp:.4f}] Roll: {roll:>9.4f} | Pitch: {pitch:>9.4f} | Yaw: {yaw:>9.4f}", end='\r', flush=True)
        except:
            print(f"Raw unparseable data: {msg.data.decode()}")

async def main():
    # Connect to a NATS server
    # 100.64.0.110 is the tailscale IP for the carrier board
    # 10.0.0.2 is the static Wireguard IP over the hotspot connection
    # 192.168.100.205 is the HaLow receiver's IP
    nc = await nats.connect("nats://192.168.100.205:4222")

    try:
        if len(sys.argv) > 1:
            if sys.argv[1] == "-c" and len(sys.argv) == 3:
                await send_command(nc, sys.argv[2])
            elif sys.argv[1] == "-t":
                await request_telem(nc)
            else:
                print("Usage: python nats_pub.py [-c 'command'] | [-t]")

    except TimeoutError:
        print("Timed out waiting for response.")
    except NoRespondersError:
        print("No one is listening on the given subject.")
    finally:
        # Ensure all messages have reached the server
        await nc.flush()
        await nc.close()

if __name__ == '__main__':
    asyncio.run(main())
