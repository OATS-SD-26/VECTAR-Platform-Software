from pymavlink import mavutil
import math
import time
import asyncio
STARTING_ALT = None
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

async def get_telem(drone):
    # Send heartbeat so ArduPilot knows we are still connected
    drone.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0, 0, 0
    )

    attitude = None
    position = None

    try:
        for _ in range(50):  # Increased attempts for debugging
            msg = drone.recv_match(blocking=False)
            if msg:
                msg_type = msg.get_type()
                print(f"Received message: {msg_type}")
                if msg_type == 'ATTITUDE' and attitude is None:
                    attitude = {
                        "roll": math.degrees(msg.roll),
                        "pitch": math.degrees(msg.pitch),
                        "yaw": math.degrees(msg.yaw),
                    }
                elif msg_type == 'GLOBAL_POSITION_INT' and position is None:
                    position = {
                        "lat": msg.lat / 1e7,  # degrees
                        "lon": msg.lon / 1e7,  # degrees
                        "alt": msg.relative_alt / 1e3,  # meters above home
                        "hdg": msg.hdg / 100.0,  # degrees (0-360)
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
        0,  # rate ignored
        0   # STOP STREAM
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
        mode_id, 0, 0, 0, 0, 0
    )

    print(f"Switching to {mode} mode...")

async def starting_alt(drone, lock): #get the starting height and set to a global variable
    global STARTING_ALT
    if STARTING_ALT is not None:
        print(f"Starting altitude is already set at {STARTING_ALT}m")
        return STARTING_ALT
    print("Finding starting altitude")
    async with lock:
        t = await get_telem(drone)
    if t is None:
        print("Error with get telementary data")
        return None
    else:
        start_alt = t["alt"]
        print("Extracted intial altitude")
    if start_alt == -1:
        print("Unable to real precise starting altitude")
        return None
    else:
        STARTING_ALT = start_alt
        print(f"Sucessfully extracted starting altitude which is set to {start_alt} m")
        return STARTING_ALT

def gps_offset_meters(lat, lon, distance_m, bearing_deg):
    R = 6371000  # Earth radius in meters

    lat1 = math.radians(lat)
    lon1 = math.radians(lon)
    bearing = math.radians(bearing_deg)

    angular_distance = distance_m / R

    lat2 = math.asin(
        math.sin(lat1) * math.cos(angular_distance)
        + math.cos(lat1) * math.sin(angular_distance) * math.cos(bearing)
    )

    lon2 = lon1 + math.atan2(
        math.sin(bearing) * math.sin(angular_distance) * math.cos(lat1),
        math.cos(angular_distance) - math.sin(lat1) * math.sin(lat2)
    )

    return math.degrees(lat2), math.degrees(lon2)

def distance_between_points(lat1,lon1,lat2,lon2):
    R = 6371000
    lat1 = math.radians(lat1)
    lon1 = math.radians(lon1)
    lat2 = math.radians(lat2)
    lon2 = math.radians(lon2)

    dlat = lat2 - lat1
    dlon = lon2 - lon1

    a = (
        math.sin(dlat / 2) ** 2
        + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    )

    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    return R * c
    
def send_gps_target(drone, target_lat, target_lon, target_alt):
    drone.mav.set_position_target_global_int_send(
        0,
        drone.target_system,
        drone.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        3576,
        int(target_lat * 1e7),
        int(target_lon * 1e7),
        target_alt,
        0, 0, 0,
        0, 0, 0,
        0,
        0
    )

    
async def current_height(drone, lock): #get the current height helper function
    print("Finding current altitude")
    async with lock:
        t = await get_telem(drone)
    if t is None:
        print("Error with get telementary data")
        return None
    else:
        current_alt = t["alt"]
        print("Extracted current altitude")
    if current_alt == -1:
        print("Unable to real precise current altitude")
        return None
    else:
        print(f"Sucessfully extracted current altitude which is set to {current_alt} m")
        return current_alt

async def move_gps(drone, target_lat, target_lon, target_alt,lock,timeout,accept_radius): #Movement based on gps coords
    print(f"Moving to GPS target: "f"lat={target_lat}, lon={target_lon}, alt={target_alt} m")
    async with lock:
        clear_all_overrides(drone)
        set_mode(drone, "GUIDED")
    start_time = time.time()
    while time.time() - start_time < timeout:
        async with lock:
            drone.mav.set_position_target_global_int_send(
                0,
                drone.target_system,
                drone.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                3576,
                int(target_lat * 1e7),
                int(target_lon * 1e7),
                target_alt,
                0, 0, 0,
                0, 0, 0,
                0,
                0)
        await asyncio.sleep(0.5)
        async with lock:
            telemetry = await get_telem(drone)
        if telemetry is None:
            continue
        current_lat = telemetry["lat"]
        current_lon = telemetry["lon"]
        if current_lat == -1 or current_lon == -1:
            continue
        remaining_distance = distance_between_points(current_lat,current_lon,target_lat,target_lon)
        print(f"Distance remaining: {remaining_distance} m")
        if remaining_distance <= accept_radius:
            print("GPS target reached")
            return True
    print("GPS movement timed out")
    return False


async def move_rel(drone, direction, distance, lock): #Movement based on distance and direction
    async with lock:
        telemetry = await get_telem(drone)
    if telemetry is None:
        print("Could not obtain telemetry")
        return False
    current_lat = telemetry["lat"]
    current_lon = telemetry["lon"]
    current_alt = telemetry["alt"]
    current_heading = telemetry["hdg"]
    if (current_lat == -1 or current_lon == -1 or current_alt == -1 or current_heading == -1):
        print("Invalid GPS, altitude, or heading data")
        return False
    direction_offsets = {
        "forward": 0,
        "right": 90,
        "back": 180,
        "left": -90
    }
    if direction not in direction_offsets:
        print(f"Unknown movement direction: {direction}")
        return False
    target_bearing = (current_heading + direction_offsets[direction]) % 360
    target_lat, target_lon = gps_offset_meters(current_lat,current_lon,distance,target_bearing)
    print(f"Current heading: {current_heading}°")
    print(f"Movement bearing: {target_bearing}°")
    print(f"Target GPS: {target_lat}, {target_lon}")
    return await move_gps(drone,target_lat,target_lon,current_alt,lock,20, min(1.0, distance * 0.25))
    
    
async def increase_height(drone, target_height, lock, throttle_val=1550, timeout=10): #To increase the height in meters
    refference_alt = await current_height(drone, lock)
    if refference_alt is None:
        print("Could not get current height")
        return None
    start_time = time.time()

    while time.time()-start_time < timeout:
        current_alt = await current_height(drone, lock)

        if current_alt is None:
            await asyncio.sleep(0.1)
            continue
    
        delta_alt = current_alt - refference_alt
        print(f"Drone being moved {delta_alt}m")
        if delta_alt >= target_height:
            print(f"Increase of {delta_alt} has been achived")
            async with lock:
                clear_all_overrides(drone)
            return True
        await movement(drone,duration=0.2,lock=lock,throttle_val=throttle_val)
    print("Increase height timeout has been reached")
    async with lock:
        clear_all_overrides(drone)
    return False

async def hold_pos(drone,lock): # Keeps pitch, yaw, roll and altitude all constant
    print("Switching to lotier mode")
    async with lock:
        set_mode(drone,"LOITER")
        clear_all_overrides(drone)
    return True

async def decrease_height(drone, target_height, lock, throttle_val=1450, timeout=10): #To decrease the height in meters
    refference_alt = await current_height(drone, lock)
    if refference_alt is None:
        print("Could not get current height")
        return None
    start_time = time.time()

    while time.time()-start_time < timeout:
        current_alt = await current_height(drone, lock)

        if current_alt is None:
            await asyncio.sleep(0.1)
            continue
    
        delta_alt = refference_alt - current_alt
        print(f"Drone being moved {delta_alt}m")
        if delta_alt >= target_height:
            print(f"Decrease of {delta_alt} has been achived")
            async with lock:
                clear_all_overrides(drone)
            return True
        await movement(drone,duration=0.2,lock=lock,throttle_val=throttle_val)
    print("Decrease height timeout has been reached")
    async with lock:
        clear_all_overrides(drone)
    return False

async def arm_vehicle(drone, timeout=10):
    print("Sending arming command...")

    # master.target_system is the ID of the Cube (usually 1)
    # master.target_component is the ID of the flight controller (usually 1)
    drone.mav.command_long_send(
        drone.target_system,
        drone.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,  # 1 to ARM, 0 to DISARM
        0, 0, 0, 0, 0, 0
    )

    # Wait until the vehicle acknowledges it is armed
    print("Waiting for vehicle to arm...")

    start_time = time.time()

    # drone.motors_armed_wait() # Don't want to use this since it's not async-friendly
    # arming has a timeout before it calls itself again, allows for no flying commands to work without having to be armed
    while time.time() - start_time < timeout:
        msg = drone.recv_match(type='HEARTBEAT', blocking=False)
        if msg:
            if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:  # This checks if the armed flag is set
                print("VEHICLE ARMED!")
                return True
        await asyncio.sleep(0.1)  # Give NATS control back in between checks for arm

    print("VEHCILE NOT ABLE TO BE ARMED!")
    return False

async def disarm_vehicle(drone):
    print("Sending disarm command...")

    for attempt in range(30):
        # Set throttle to minimum value and center virtual sticks
        drone.mav.rc_channels_override_send(
            drone.target_system,
            drone.target_component,
            1500, 1500, 1000, 1500, 65535, 65535, 65535, 65535
        )

        # Send disarm command using FORCE flag
        drone.mav.command_long_send(
            drone.target_system,
            drone.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,      # 0 to DISARM
            21196,  # This is used to FORCE disarm
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
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # Frame of reference
        0b0000111111000111,  # Type mask: only use velocities
        0, 0, 0,             # x, y, z positions (ignored)
        vx, vy, vz,          # x, y, z velocities
        0, 0, 0,             # x, y, z acceleration (ignored)
        0, 0                 # yaw, yaw_rate (ignored)
    )

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
    #     drone.target_system,
    #     drone.target_component,
    #     mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
    #     0,                 # Confirmation
    #     1,                 # Param 1: Motor instance number
    #     1,                 # Param 2: Throttle type (1 = percentage)
    #     throttle_val,      # Param 3: Throttle value
    #     duration,          # Param 4: Timeout in seconds
    #     0,                 # Param 5: Motor count (for multiple motors)
    #     0, 0               # Param 6, 7: Unused
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
                65535, 65535, 65535, 65535  # Chans 5-8
            )

        await asyncio.sleep(0.1)  # Send at 10Hz

def clear_all_overrides(drone):
    print("Releasing all RC overrides to 0...")
    drone.mav.rc_channels_override_send(
        drone.target_system,
        drone.target_component,
        0, 0, 0, 0, 0, 0, 0, 0
    )

async def movement(
    drone,
    duration,
    lock,
    roll_val=65535,
    pitch_val=65535,
    throttle_val=65535,
    yaw_val=65535
):
    print(f"RC override for {duration} seconds | " f"roll = {roll_val}, pitch = {pitch_val}, throttle = {throttle_val}, yaw = {yaw_val}")
    end_time = time.time() + duration

    while(time.time() < end_time):
        async with lock:
            drone.mav.rc_channels_override_send(
                drone.target_system,
                drone.target_component,
                roll_val,      # Chan 1 (Roll)
                pitch_val,     # Chan 2 (Pitch)
                throttle_val,  # Chan 3 (Throttle)
                yaw_val,       # Chan 4 (Yaw)
                65535, 65535, 65535, 65535  # Chans 5-8
            )
        await asyncio.sleep(0.1)  # Send at 10Hz

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

    print("")  # This is to handle the telem text sticking around when stopping the program
