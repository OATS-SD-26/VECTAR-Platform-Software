import time
from pymavlink import mavutil

PORT = "/dev/ttyS4"
BAUD = 57600

print(f"Connecting to {PORT} at {BAUD} baud...")
drone = mavutil.mavlink_connection(PORT, BAUD)
drone.source_system = 255

print("Waiting for heartbeat...")
drone.wait_heartbeat()
print("Heartbeat received. Starting latency test...\n")

def measure_latency():
    # 1. Record time right before sending
    start_time = time.time()
    
    # 2. Ask for the parameter
    drone.mav.param_request_read_send(1, 1, b'SYSID_THISMAV', -1)
    
    # 3. Wait for the exact response message
    msg = drone.recv_match(type='PARAM_VALUE', blocking=True, timeout=2.0)
    
    if msg:
        # 4. Record time immediately upon receiving it
        end_time = time.time()
        
        # Calculate in milliseconds
        latency_ms = (end_time - start_time) * 1000.0
        print(f"Round-trip latency: {latency_ms:.2f} ms")
    else:
        print("Request timed out (packet lost).")

try:
    # Run the test once a second until you press Ctrl+C
    while True:
        measure_latency()
        time.sleep(1)
        
except KeyboardInterrupt:
    print("\nLatency test stopped.")