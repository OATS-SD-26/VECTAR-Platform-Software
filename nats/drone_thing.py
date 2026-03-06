from pymavlink import mavutil
import math

def initialize_telem():
	PORT = "/dev/ttyS4"
	BAUD = 57600

	drone = mavutil.mavlink_connection(PORT, BAUD)

	print("Waiting for heartbeat...")
	drone.wait_heartbeat()

	print("Heartbeat from system (system %u component %u)" % (drone.target_system, drone.target_component))


	drone.mav.request_data_stream_send(
		drone.target_system,
		drone.target_component,
		mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
		2,  # Hz
		1
	)

	'''
	drone.mav.request_data_stream_send(
		drone.target_system,
		drone.target_component,
		mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
		0,   # rate ignored
		0    # STOP STREAM
	)
	'''

	return drone

def get_telem(drone):
	'''
	msg = drone.recv_match(blocking=True)
	if msg:
		print(msg)

	'''
	
	msg = drone.recv_match(type='ATTITUDE', blocking=True)
	if not msg:
		return {"roll": -1, "pitch": -1, "yaw": -1}
	
	roll = math.degrees(msg.roll)
	pitch = math.degrees(msg.pitch)
	yaw = math.degrees(msg.yaw)

	return{"roll": roll, "pitch": pitch, "yaw": yaw}
	
