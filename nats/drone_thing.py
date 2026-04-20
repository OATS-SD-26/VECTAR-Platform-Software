from pymavlink import mavutil
import math
import time
import asyncio

async def initialize_telem(drone):
	print("Waiting for heartbeat...")
	try:
		while True:
			msg = drone.recv_match(type='HEARTBEAT', blocking=False)
			if msg:
				print("Heartbeat from system (system %u component %u)" % (drone.target_system, drone.target_component))
				drone.mav.request_data_stream_send(
					drone.target_system,
					drone.target_component,
					mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
					10,  # Hz
					1
				)
				return
			await asyncio.sleep(0.1)
	except Exception as e:
		print(f"Error initializing telemetry: {e}")
		return

	'''
	drone.mav.request_data_stream_send(
		drone.target_system,
		drone.target_component,
		mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
		0,   # rate ignored
		0    # STOP STREAM
	)
	'''

async def get_telem(drone):
	'''
	msg = drone.recv_match(blocking=True)
	if msg:
		print(msg)

	'''
    # Send heartbeat so ArduPilot knows we are still connected
	drone.mav.heartbeat_send(
		mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0, 0, 0
    )

	attitude = None
	position = None

	try:
		for _ in range(10):
			if attitude is None:
				msg = drone.recv_match(type='ATTITUDE', blocking=False)
				if msg:
					attitude = {
						"roll":  math.degrees(msg.roll),
						"pitch": math.degrees(msg.pitch),
						"yaw":   math.degrees(msg.yaw),
					}

			if position is None:
				msg = drone.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
				if msg:
					position = {
						"lat": msg.lat / 1e7,        # degrees
						"lon": msg.lon / 1e7,         # degrees
						"alt": msg.relative_alt / 1e3, # meters above home
						"hdg": msg.hdg / 100.0,        # degrees (0-360)
					}

			if attitude and position:
				break

			await asyncio.sleep(0.02)
	except Exception as e:
		print(f"Error receiving telemetry: {e}")
		return None

	return {
		"roll": attitude["roll"] if attitude else -1,
		"pitch": attitude["pitch"] if attitude else -1,
		"yaw": attitude["yaw"] if attitude else -1,
		"lat": position["lat"] if position else -1,
		"lon": position["lon"] if position else -1,
		"alt": position["alt"] if position else -1,
		"hdg": position["hdg"] if position else -1,
	}

def stop_telem(drone):
	# Stop orientation stream
	drone.mav.request_data_stream_send(
		drone.target_system,
		drone.target_component,
		mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
		0,   # rate ignored
		0    # STOP STREAM
	)

def set_mode(drone, mode):
	# Check if the mode is available in the mapping
	if mode not in drone.mode_mapping():
		print(f"Unknown mode : {mode}")
		return

	mode_id = drone.mode_mapping()[mode]

	# Send the command to change mode
	drone.mav.command_long_send(
		drone.target_system, 
		drone.target_component,
		mavutil.mavlink.MAV_CMD_DO_SET_MODE,
		0,
		mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
		mode_id, 0, 0, 0, 0, 0)
	
	print(f"Switching to {mode} mode...")

async def arm_vehicle(drone):
	print("Sending arming command...")
	
	# master.target_system is the ID of the Cube (usually 1)
	# master.target_component is the ID of the flight controller (usually 1)
	drone.mav.command_long_send(
		drone.target_system,
		drone.target_component,
		mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
		0,
		1, # 1 to ARM, 0 to DISARM
		0, 0, 0, 0, 0, 0
	)

	# Wait until the vehicle acknowledges it is armed
	print("Waiting for vehicle to arm...")
	# drone.motors_armed_wait() # Don't want to use this since it's not async-friendly
	while True:
		msg = drone.recv_match(type='HEARTBEAT', blocking=False)
		if msg:
			if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED: # This checks if the armed flag is set
				print("VEHICLE ARMED!")
				break
		await asyncio.sleep(0.1) # Give NATS control back in between checks for arm

async def disarm_vehicle(drone):
	print("Sending disarm command...")

	for attempt in range(30):
		# Set throttle to minimum value and center virtual sticks
		drone.mav.rc_channels_override_send(
			drone.target_system, drone.target_component,
			1500, 1500, 1000, 1500, 65535, 65535, 65535, 65535
		)

		# Send disarm command using FORCE flag
		drone.mav.command_long_send(
			drone.target_system,
			drone.target_component,
			mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
			0,
			0, # 0 to DISARM
			21196, # This is used to FORCE disarm
			0, 0, 0, 0, 0
		)

		# Check if successfully disarmed
		msg = drone.recv_match(type='HEARTBEAT', blocking=False)
		if msg:
			if not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
				print("VEHICLE DISARMED!")
				return
		await asyncio.sleep(0.1)

	print("WARNING: Drone refused to disarm after 3 seconds!")

'''
This only worked in the GUIDED mode, which doesn't work for now
'''
def send_velocity_command(drone, vx, vy, vz):
	"""
	vx: m/s North
	vy: m/s East
	vz: m/s Down (Positive is DOWN, so -1.0 is 1m/s UP)
	"""
	print("Velocity: ", vx, vy, vz)
	drone.mav.set_position_target_local_ned_send(
		0,       # time_boot_ms (not used)
		drone.target_system, 
		drone.target_component,
		mavutil.mavlink.MAV_FRAME_LOCAL_NED, # Frame of reference
		0b0000111111000111, # Type mask: only use velocities
		0, 0, 0,            # x, y, z positions (ignored)
		vx, vy, vz,         # x, y, z velocities
		0, 0, 0,            # x, y, z acceleration (ignored)
		0, 0)               # yaw, yaw_rate (ignored)

'''
This is to test throttle in STABLIZE mode
'''
async def throttle_continuous(drone, throttle_val, duration, lock):
	"""
	throttle_pwm: 1000 (off) to 2000 (full)
	duration: seconds to hold this throttle
	"""
	print(f"Driving throttle to {throttle_val} for {duration} seconds...")
	end_time = time.time() + duration

	# drone.mav.command_long_send(
	# 	drone.target_system,
	# 	drone.target_component,
	# 	mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
	# 	0,                 # Confirmation
	# 	1,       # Param 1: Motor instance number
	# 	1,                 # Param 2: Throttle type (1 = percentage)
	# 	throttle_val,  # Param 3: Throttle value
	# 	duration,      # Param 4: Timeout in seconds
	# 	0,                 # Param 5: Motor count (for multiple motors)
	# 	0, 0               # Param 6, 7: Unused
	# )
	
	while time.time() < end_time:
		async with lock:
			# Channel 3 is standard for Throttle in ArduPilot
			# We set other channels to 65535 to tell the Cube "ignore these, use current state"
			drone.mav.rc_channels_override_send(
				drone.target_system,
				drone.target_component,
				65535,        # Chan 1 (Roll)
				65535,        # Chan 2 (Pitch)
				throttle_val, # Chan 3 (Throttle) - THIS IS THE ONE
				65535,        # Chan 4 (Yaw)
				65535, 65535, 65535, 65535 # Chans 5-8
			)
			drone.recv_match(blocking=False)
		await asyncio.sleep(0.1) # Send at 10Hz

def clear_all_overrides(drone):
	print("Releasing all RC overrides to 0...")
	drone.mav.rc_channels_override_send(
		drone.target_system,
		drone.target_component,
		0, 0, 0, 0, 0, 0, 0, 0
	)
	

if __name__ == "__main__":
	PORT = "/dev/ttyS4"
	BAUD = 57600
	drone = mavutil.mavlink_connection(PORT, BAUD)
	drone.source_system = 255

	# Test motors
	set_mode(drone, 'STABILIZE')
	arm_vehicle(drone)
	throttle_continuous(drone, 1500, 5)
	disarm_vehicle(drone)

	# Test telem
	drone = initialize_telem(drone)
	while True:
		try:
			t = get_telem(drone)
			print(f"Roll: {t["roll"]:.4f} | Pitch: {t["pitch"]:.4f} | Yaw: {t["yaw"]:.4f}", end='\r', flush=True)
			print(f"Lat: {t["lat"]:.4f} | Lon: {t["lon"]:.4f} | Alt: {t["alt"]:.4f}", end='\r', flush=True)


		except KeyboardInterrupt:
			break
	print("") # This is to handle the telem text sticking around when stopping the program