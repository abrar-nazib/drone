#############DEPENDENCIES#######################


from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException, mavutil
import time
import socket

# import exceptions
import math

##############FUNCTIONS##########################


##Function to arm the drone props and takeoff at targetHeight (m)


def arm_and_takeoff(targetHeight):

    while vehicle.is_armable != True:
        print("Waiting for vehicle to become armable.")
        time.sleep(1)
    print("Vehicle is now armable")

    vehicle.mode = VehicleMode("GUIDED")

    while vehicle.mode != "GUIDED":
        print("Waiting for drone to enter GUIDED flight mode")
        time.sleep(1)
    print("Vehicle now in GUIDED MODE. Have fun!!")

    vehicle.armed = True
    while vehicle.armed == False:
        print("Waiting for vehicle to become armed.")
        time.sleep(1)
    print("Look out! Virtual props are spinning!!")

    vehicle.simple_takeoff(targetHeight)

    while True:
        print("Current Altitude: %d" % vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= 0.95 * targetHeight:
            break
        time.sleep(1)
    print("Target altitude reached!!")

    return None


def send_global_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,  # time_boot_ms (not used)
        0,
        0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # frame
        0b0000111111000111,  # type_mask (only speeds enabled)
        0,  # lat_int - X Position in WGS84 frame in 1e7 * meters
        0,  # lon_int - Y Position in WGS84 frame in 1e7 * meters
        0,  # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
        # altitude above terrain if GLOBAL_TERRAIN_ALT_INT
        velocity_x,  # X velocity in NED frame in m/s
        velocity_y,  # Y velocity in NED frame in m/s
        velocity_z,  # Z velocity in NED frame in m/s
        0,
        0,
        0,  # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0,
        0,
    )  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to vehicle on 1 Hz cycle
    for x in range(0, duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)


############MAIN EXECUTABLE#############


####sim_vehicle.py opens up port on localhost:14550


vehicle = connect("/dev/ttyACM0", wait_ready=True)


####Arm the drone and takeoff into the air at 5 meters


# Reduce the takeoff speed to 1m/s. Default is 5m/s.
vehicle.airspeed = 1

arm_and_takeoff(2)
print("Vehicle reached target altitude")


# Delay for 10 seconds
time.sleep(5)

# Move forward 10 metres from current location
print("Moving Forward")
send_global_velocity(1, 0, 0, 2)
send_global_velocity(0, 0, 0, 2)
####Once drone reaches target altitude, change mode to LAND


vehicle.mode = VehicleMode("LAND")
while vehicle.mode != "LAND":
    print("Waiting for drone to enter LAND mode")
    time.sleep(1)
print("Vehicle now in LAND mode. Will touch ground shortly.")
