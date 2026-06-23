from pymavlink import mavutil
import math
import time
import asyncio
STARTING_POSITION = None

async def initialize_telem(drone):
    print("Waiting for heartbeat...")
    try:
        while True:
            msg = drone.recv_match(type="HEARTBEAT", blocking=False)
            if msg:
                print(f"Heartbeat from system {drone.target_system}, component {drone.target_component}")
                drone.mav.request_data_stream_send(drone.target_system, drone.target_component, mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 10, 1)
                drone.mav.request_data_stream_send(drone.target_system, drone.target_component, mavutil.mavlink.MAV_DATA_STREAM_POSITION, 10, 1)
                return True
            await asyncio.sleep(0.1)
    except Exception as error:
        print(f"Error initializing telemetry: {error}")
        return False

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
                        "hdg": (-1
                                if msg.hdg == 65535
                                else msg.hdg / 100.0
                               ),
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

async def set_mode(drone, mode, timeout=5):
    mode_mapping = drone.mode_mapping() or {}
    if mode not in mode_mapping:
        print(f"Unknown mode: {mode}")
        return False

    mode_id = mode_mapping[mode]
    drone.mav.command_long_send(drone.target_system, drone.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, 0, 0, 0, 0, 0)
    print(f"Requested {mode} mode")

    start_time = time.monotonic()
    while time.monotonic() - start_time < timeout:
        msg = drone.recv_match(type="HEARTBEAT", blocking=False)
        if msg and msg.custom_mode == mode_id:
            print(f"Vehicle entered {mode} mode")
            return True
        await asyncio.sleep(0.1)

    print(f"Vehicle did not confirm {mode} mode")
    return False

    print(f"Switching to {mode} mode...")

async def starting_alt(drone, lock):
    position = await starting_position(drone, lock)

    if position is None:
        return None

    return position["alt"]
        
async def starting_position(drone, lock):
    global STARTING_POSITION
    if STARTING_POSITION is not None:
        print(f"Starting position already stored: "f"lat={STARTING_POSITION['lat']}, "f"lon={STARTING_POSITION['lon']}, "f"alt={STARTING_POSITION['alt']} m")
        return STARTING_POSITION
    print("Finding starting position")
    async with lock:
        telemetry = await get_telem(drone)
    if telemetry is None:
        print("Could not obtain starting-position telemetry")
        return None
    start_lat = telemetry["lat"]
    start_lon = telemetry["lon"]
    start_alt = telemetry["alt"]

    if start_lat == -1 or start_lon == -1 or start_alt == -1:
        print("Could not obtain valid starting coordinates")
        return None

    STARTING_POSITION = {
        "lat": start_lat,
        "lon": start_lon,
        "alt": start_alt
    }

    print(
        f"Starting position stored: "
        f"lat={start_lat}, lon={start_lon}, alt={start_alt} m"
    )

    return STARTING_POSITION
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

def calculate_horizontal_accept_radius(distance):
    return min(1.0, distance * 0.25)

def calculate_vertical_accept_radius(height_change):
    return min(0.3, height_change * 0.25)

async def move_gps(drone, target_lat, target_lon, target_alt, lock, timeout, horizontal_accept_radius, vertical_accept_radius):
    print(f"Moving to GPS target: lat={target_lat}, lon={target_lon}, alt={target_alt} m")
    start_time = time.monotonic()
    while time.monotonic() - start_time < timeout:
        async with lock:
            send_gps_target(drone, target_lat, target_lon, target_alt)
        await asyncio.sleep(0.5)
        async with lock:
            telemetry = await get_telem(drone)
        if telemetry is None:
            continue
        current_lat = telemetry["lat"]
        current_lon = telemetry["lon"]
        current_alt = telemetry["alt"]
        if current_lat == -1 or current_lon == -1 or current_alt == -1:
            print("Waiting for valid position telemetry")
            continue
        horizontal_error = distance_between_points(current_lat, current_lon, target_lat, target_lon)
        vertical_error = abs(current_alt - target_alt)
        print(f"Horizontal error: {horizontal_error:.2f} m | Vertical error: {vertical_error:.2f} m")
        if horizontal_error <= horizontal_accept_radius and vertical_error <= vertical_accept_radius:
            print("GPS target reached")
            return True
    print("GPS movement timed out")
    return False

async def move_rel(drone, direction, distance, lock):
    if distance <= 0:
        print("Movement distance must be positive")
        return False
    async with lock:
        telemetry = await get_telem(drone)
    if telemetry is None:
        print("Could not obtain telemetry")
        return False
    current_lat = telemetry["lat"]
    current_lon = telemetry["lon"]
    current_alt = telemetry["alt"]
    current_heading = telemetry["hdg"]
    if current_lat == -1 or current_lon == -1 or current_alt == -1 or current_heading == -1:
        print("Invalid GPS, altitude, or heading data")
        return False
    direction_offsets = {"forward": 0, "right": 90, "back": 180, "left": -90}
    if direction not in direction_offsets:
        print(f"Unknown movement direction: {direction}")
        return False
    target_bearing = (current_heading + direction_offsets[direction]) % 360
    target_lat, target_lon = gps_offset_meters(current_lat, current_lon, distance, target_bearing)
    horizontal_accept_radius = calculate_horizontal_accept_radius(distance)
    print(f"Current heading: {current_heading}°")
    print(f"Movement bearing: {target_bearing}°")
    print(f"Target GPS: {target_lat}, {target_lon}")
    return await move_gps(drone, target_lat, target_lon, current_alt, lock, 20, horizontal_accept_radius, 0.3)

async def increase_height(drone, target_height, lock, timeout=20):
    if target_height <= 0:
        print("Height increase must be positive")
        return False
    async with lock:
        telemetry = await get_telem(drone)
    if telemetry is None:
        print("Could not obtain telemetry")
        return False
    current_lat = telemetry["lat"]
    current_lon = telemetry["lon"]
    current_alt = telemetry["alt"]
    if current_lat == -1 or current_lon == -1 or current_alt == -1:
        print("Invalid GPS or altitude data")
        return False
    target_alt = current_alt + target_height
    vertical_accept_radius = calculate_vertical_accept_radius(target_height)

    print(f"Increasing altitude from {current_alt:.2f} m to {target_alt:.2f} m")
    return await move_gps(drone, current_lat, current_lon, target_alt, lock, timeout, 1.0, vertical_accept_radius)

async def hold_pos(drone, lock):
    print("Switching to loiter mode")
    async with lock:
        clear_all_overrides(drone)
        loitered = await set_mode(drone, "LOITER")
    return loitered

async def decrease_height(drone, target_height, lock, timeout=20):
    if target_height <= 0:
        print("Height decrease must be positive")
        return False
    async with lock:
        telemetry = await get_telem(drone)
    if telemetry is None:
        print("Could not obtain telemetry")
        return False
    current_lat = telemetry["lat"]
    current_lon = telemetry["lon"]
    current_alt = telemetry["alt"]
    if current_lat == -1 or current_lon == -1 or current_alt == -1:
        print("Invalid GPS or altitude data")
        return False
    target_alt = current_alt - target_height
    if target_alt <= 0:
        print("Target altitude is at or below home altitude; use the landing command instead")
        return False
    vertical_accept_radius = calculate_vertical_accept_radius(target_height)
    print(f"Decreasing altitude from {current_alt:.2f} m to {target_alt:.2f} m")
    return await move_gps(drone, current_lat, current_lon, target_alt, lock, timeout, 1.0, vertical_accept_radius)

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

async def takeoff_vehicle(drone, lock, takeoff_height=1.0, timeout=20, ack_timeout=3):
    if takeoff_height <= 0:
        return {"status": "error", "reason": "invalid_takeoff_height", "message": "Takeoff height must be positive.", "airborne": False}
    start_alt = await starting_alt(drone, lock)
    if start_alt is None or start_alt == -1:
        return {"status": "error", "reason": "starting_altitude_unavailable", "message": "Could not obtain a valid starting altitude.", "airborne": False}
    target_alt = start_alt + takeoff_height
    vertical_accept_radius = calculate_vertical_accept_radius(takeoff_height)
    print(f"Starting altitude: {start_alt:.2f} m")
    print(f"Takeoff target: {target_alt:.2f} m")
    print(f"Vertical acceptance radius: {vertical_accept_radius:.2f} m")
    async with lock:
        armed = await arm_vehicle(drone)
        if not armed:
            return {"status": "error", "reason": "arming_failed", "message": "Vehicle could not be armed.", "airborne": False, "start_alt": start_alt, "target_alt": target_alt}
        drone.mav.command_long_send(
            drone.target_system,
            drone.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0,
            0,
            0,
            float("nan"),
            0,
            0,
            target_alt
        )
        print(f"Takeoff command sent for {target_alt:.2f} m above home")
        ack_result = None
        ack_start = time.monotonic()
        while time.monotonic() - ack_start < ack_timeout:
            ack = drone.recv_match(type="COMMAND_ACK", blocking=False)
            if ack and ack.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
                ack_result = ack.result
                break
            await asyncio.sleep(0.1)
    accepted_results = (mavutil.mavlink.MAV_RESULT_ACCEPTED,mavutil.mavlink.MAV_RESULT_IN_PROGRESS)

    if ack_result is not None and ack_result not in accepted_results:
        ack_messages = {
            mavutil.mavlink.MAV_RESULT_TEMPORARILY_REJECTED: "ArduPilot temporarily rejected the takeoff command.",
            mavutil.mavlink.MAV_RESULT_DENIED: "ArduPilot denied the takeoff command.",
            mavutil.mavlink.MAV_RESULT_UNSUPPORTED: "ArduPilot reported that the takeoff command is unsupported.",
            mavutil.mavlink.MAV_RESULT_FAILED: "ArduPilot reported that the takeoff command failed.",
            mavutil.mavlink.MAV_RESULT_CANCELLED: "The takeoff command was cancelled."
        }
        return {"status": "error","reason": "takeoff_rejected","message": ack_messages.get(ack_result, f"Takeoff was rejected with MAVLink result {ack_result}."),
            "ack_result": ack_result,"airborne": False,"start_alt": start_alt,"target_alt": target_alt}

    if ack_result is None:
        print("No takeoff acknowledgement received; monitoring altitude anyway")
    else:
        print(f"Takeoff command accepted with MAVLink result {ack_result}")

    monitor_start = time.monotonic()
    current_alt = start_alt
    received_valid_altitude = False
    climbed = False

    while time.monotonic() - monitor_start < timeout:
        async with lock:
            telemetry = await get_telem(drone)
            armed = drone.motors_armed()
        if telemetry is None or telemetry["alt"] == -1:
            await asyncio.sleep(0.2)
            continue
        received_valid_altitude = True
        current_alt = telemetry["alt"]
        if current_alt >= start_alt + vertical_accept_radius:
            climbed = True
        print(f"Takeoff altitude: {current_alt:.2f} m / {target_alt:.2f} m")
        if not armed:
            return {"status": "error","reason": "disarmed_during_takeoff","message": "Vehicle became disarmed during takeoff.","airborne": climbed,
                "start_alt": start_alt,"target_alt": target_alt,"current_alt": current_alt,"ack_result": ack_result}
        
        if current_alt >= target_alt - vertical_accept_radius:
            return {"status": "success","reason": "takeoff_complete" if ack_result is not None else "takeoff_complete_no_ack","message": "Preset takeoff altitude reached.",
                    "airborne": True,"start_alt": start_alt,"target_alt": target_alt,"current_alt": current_alt,"ack_result": ack_result}
        await asyncio.sleep(0.2)

    if not received_valid_altitude:
        reason = "takeoff_telemetry_unavailable"
        message = "No valid altitude telemetry was received during takeoff."
    elif climbed:
        reason = "takeoff_timeout_airborne"
        message = "Vehicle climbed but did not reach the preset takeoff altitude."
    else:
        reason = "takeoff_timeout_grounded"
        message = "Takeoff timed out without detecting a climb."

    return {
        "status": "error","reason": reason,"message": message,"airborne": climbed,"start_alt": start_alt,
        "target_alt": target_alt,"current_alt": current_alt,"ack_result": ack_result}

async def land_at_position(drone, lock, target_position, approach_timeout=30, landing_timeout=60, horizontal_accept_radius=1.0):
    global STARTING_POSITION
    if target_position is None:
        target_position = STARTING_POSITION
    if target_position is None:
        return {"status": "error","reason": "landing_target_unavailable","message": "No landing position has been stored.","airborne": True}
    try:
        target_lat = float(target_position["lat"])
        target_lon = float(target_position["lon"])
        target_ground_alt = float(target_position["alt"])
    except (KeyError, TypeError, ValueError):
        return {"status": "error","reason": "invalid_landing_target","message": "Landing target must contain valid latitude, longitude, and altitude values.","airborne": True}
    async with lock:
        telemetry = await get_telem(drone)
    if telemetry is None:
        return {
            "status": "error",
            "reason": "landing_telemetry_unavailable",
            "message": "Could not obtain telemetry before landing.",
            "airborne": True
        }

    current_lat = telemetry["lat"]
    current_lon = telemetry["lon"]
    current_alt = telemetry["alt"]

    if current_lat == -1 or current_lon == -1 or current_alt == -1:
        return {
            "status": "error",
            "reason": "invalid_landing_telemetry",
            "message": "Valid GPS and altitude telemetry are required for landing.",
            "airborne": True
        }

    approach_distance = distance_between_points(current_lat,current_lon,target_lat,target_lon)

    print(f"Landing target: lat={target_lat}, lon={target_lon}")
    print(f"Distance from landing target: {approach_distance:.2f} m")
    print(f"Maintaining approach altitude: {current_alt:.2f} m")

    if approach_distance > horizontal_accept_radius:
        approached = await move_gps(
            drone,
            target_lat,
            target_lon,
            current_alt,
            lock,
            approach_timeout,
            horizontal_accept_radius,
            0.3
        )

        if not approached:
            return {
                "status": "error",
                "reason": "landing_approach_failed",
                "message": "Vehicle could not reach the landing coordinates.",
                "airborne": True,
                "target_lat": target_lat,
                "target_lon": target_lon,
                "approach_distance": approach_distance
            }
    else:
        print("Vehicle is already within the landing target radius")

    async with lock:
        clear_all_overrides(drone)
        land_mode = await set_mode(drone, "LAND")

    if not land_mode:
        return {
            "status": "error",
            "reason": "land_mode_failed",
            "message": "Vehicle reached the target, but LAND mode was not confirmed.",
            "airborne": True,
            "target_lat": target_lat,
            "target_lon": target_lon
        }

    print("LAND mode confirmed. Monitoring landing.")

    monitor_start = time.monotonic()
    current_alt = telemetry["alt"]
    received_valid_altitude = False

    while time.monotonic() - monitor_start < landing_timeout:
        async with lock:
            telemetry = await get_telem(drone)
            armed = drone.motors_armed()

        if telemetry is not None and telemetry["alt"] != -1:
            current_alt = telemetry["alt"]
            received_valid_altitude = True

            print(f"Landing altitude: {current_alt:.2f} m | "f"Stored ground altitude: {target_ground_alt:.2f} m")

        if not armed:
            STARTING_POSITION = None
            return {"status": "success","reason": "landing_complete","message": "Vehicle landed and disarmed at the target position.","airborne": False,"target_lat": target_lat,"target_lon": target_lon,"current_alt": current_alt}
        await asyncio.sleep(0.2)

    if not received_valid_altitude:
        reason = "landing_telemetry_unavailable"
        message = "LAND mode was entered, but no valid altitude telemetry was received."
    else:
        reason = "landing_timeout"
        message = "Vehicle did not confirm landing and disarming before the timeout."

    return {"status": "error","reason": reason,"message": message,"airborne": True,"target_lat": target_lat,"target_lon": target_lon,"current_alt": current_alt}


async def disarm_vehicle(drone, timeout=5):
    print("Sending normal disarm command...")
    start_time = time.monotonic()

    while time.monotonic() - start_time < timeout:
        drone.mav.command_long_send(drone.target_system, drone.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)

        msg = drone.recv_match(type="HEARTBEAT", blocking=False)
        if msg and not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            print("VEHICLE DISARMED!")
            return True

        await asyncio.sleep(0.1)

    print("Vehicle did not confirm disarming")
    return False

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

async def test_telem():
    PORT = "/dev/ttyS4"
    BAUD = 57600
    drone = mavutil.mavlink_connection(PORT, BAUD)
    drone.source_system = 255
    lock = asyncio.Lock()

    async with lock:
        initialized = await initialize_telem(drone)

    if not initialized:
        print("Telemetry initialization failed")
        return

    print("Telemetry initialized. Press Ctrl+C to stop.")

    try:
        while True:
            async with lock:
                telemetry = await get_telem(drone)

            if telemetry is not None:
                print(f"Roll: {telemetry['roll']:.2f} | Pitch: {telemetry['pitch']:.2f} | Yaw: {telemetry['yaw']:.2f}")
                print(f"Lat: {telemetry['lat']:.7f} | Lon: {telemetry['lon']:.7f} | Alt: {telemetry['alt']:.2f} | Heading: {telemetry['hdg']:.2f}")

            await asyncio.sleep(0.5)

    except KeyboardInterrupt:
        print("\nTelemetry test stopped")

if __name__ == "__main__":
    asyncio.run(test_telem())
