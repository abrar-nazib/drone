from dronekit import connect, VehicleMode, LocationGlobalRelative, mavutil, Vehicle
import time
from colorama import Fore, Back, Style
import math
import cv2
import numpy as np
# Connect to the Vehicle (in this case a simulator running the same 
# computer)

def log_stats(vehicle:Vehicle):
    print(f"[{Fore.GREEN}INFO{Style.RESET_ALL}] Autopilot Firmware version: {vehicle.version}")
    print(f"[{Fore.GREEN}INFO{Style.RESET_ALL}] Autopilot capabilities (supports ftp): {vehicle.capabilities.ftp}")
    print(f"[{Fore.GREEN}INFO{Style.RESET_ALL}] Global Location: {vehicle.location.global_frame}")
    print(f"[{Fore.GREEN}INFO{Style.RESET_ALL}] Global Location (relative altitude): {vehicle.location.global_relative_frame}")
    print(f"[{Fore.GREEN}INFO{Style.RESET_ALL}] Local Location: {vehicle.location.local_frame}")    #NED
    print(f"[{Fore.GREEN}INFO{Style.RESET_ALL}] Attitude: {vehicle.attitude}")
    print(f"[{Fore.GREEN}INFO{Style.RESET_ALL}] Velocity: {vehicle.velocity}")
    print(f"[{Fore.GREEN}INFO{Style.RESET_ALL}] GPS: {vehicle.gps_0}")
    print(f"[{Fore.GREEN}INFO{Style.RESET_ALL}] Groundspeed: {vehicle.groundspeed}")
    print(f"[{Fore.GREEN}INFO{Style.RESET_ALL}] Airspeed: {vehicle.airspeed}")
    print(f"[{Fore.GREEN}INFO{Style.RESET_ALL}] Gimbal status: {vehicle.gimbal}")
    print(f"[{Fore.GREEN}INFO{Style.RESET_ALL}] Battery: {vehicle.battery}")
    print(f"[{Fore.GREEN}INFO{Style.RESET_ALL}] EKF OK?: {vehicle.ekf_ok}")
    print(f"[{Fore.GREEN}INFO{Style.RESET_ALL}] Last Heartbeat: {vehicle.last_heartbeat}")
    print(f"[{Fore.GREEN}INFO{Style.RESET_ALL}] Rangefinder: {vehicle.rangefinder}")
    print(f"[{Fore.GREEN}INFO{Style.RESET_ALL}] Rangefinder distance: {vehicle.rangefinder.distance}")
    print(f"[{Fore.GREEN}INFO{Style.RESET_ALL}] Rangefinder voltage: {vehicle.rangefinder.voltage}")
    print(f"[{Fore.GREEN}INFO{Style.RESET_ALL}] Heading: {vehicle.heading}")
    print(f"[{Fore.GREEN}INFO{Style.RESET_ALL}] Is Armable?: {vehicle.is_armable}")
    print(f"[{Fore.GREEN}INFO{Style.RESET_ALL}] System status: {vehicle.system_status.state}")
    print(f"[{Fore.GREEN}INFO{Style.RESET_ALL}] Mode: {vehicle.mode.name}")    # settable
    print(f"[{Fore.GREEN}INFO{Style.RESET_ALL}] Armed: {vehicle.armed}")    # settable


def set_velocity(vehicle: Vehicle, velocity_x, velocity_y, velocity_z):
    """
    Move vehicle in direction based on specified velocity vectors.
    """

    print(f"[{Fore.LIGHTBLUE_EX}*{Style.RESET_ALL}] Moving vehicle in direction based on specified velocity vectors..{ velocity_x } { velocity_y } { velocity_z }")

    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    
    vehicle.send_mavlink(msg)
    print(f"[{Fore.GREEN}+{Style.RESET_ALL}] Vehicle moved")

    # # send command to vehicle on 1 Hz cycle
    # for x in range(0,duration):
    #     vehicle.send_mavlink(msg)
    #     time.sleep(1)

def set_yaw_angle(vehicle: Vehicle, heading, relative=False):
    print(f"[{Fore.LIGHTBLUE_EX}*{Style.RESET_ALL}] Setting yaw angle.. {heading}")


    current_heading = vehicle.heading
    diff = heading - current_heading

    # Put check for zero division error
    try:
        dir = diff / abs(diff)
    except ZeroDivisionError:
        dir = 1
    if(abs(diff) > 180):
        dir = -dir


    if relative:
        is_relative=1 #yaw relative to direction of travel
    else:
        is_relative=0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        dir,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)

    # while True:
    #     print(f"{Fore.LIGHTBLUE_EX}    -> Vehicle heading: {vehicle.heading}{Style.RESET_ALL}")
    #     diff = heading - vehicle.heading
        
    #     if abs(diff) <= 5:
    #         print(f"{Fore.LIGHTBLUE_EX}    -> Target heading reached{Style.RESET_ALL}")
    #         break
        # time.sleep(1)

def set_velocity_twards_yaw_heading(vehicle: Vehicle, velocity):
    """ Generate velocity_x, velocity_y, velocity_z vectors based on the current yaw heading and the velocity magnitude
    """
    print(f"[{Fore.LIGHTBLUE_EX}*{Style.RESET_ALL}] Generating velocity vectors based on the current yaw heading and the velocity magnitude.. {velocity}")

    # Vehicle.heading is in degrees
    # Convert to radians
    heading = math.radians(vehicle.heading)

    # Generate velocity vectors
    velocity_x = velocity * math.cos(heading)
    velocity_y = velocity * math.sin(heading)
    velocity_z = 0

    set_velocity(vehicle, velocity_x, velocity_y, velocity_z)

def set_yaw_velocity(vehicle: Vehicle, heading, relative=False):

    print(f"[{Fore.LIGHTBLUE_EX}*{Style.RESET_ALL}] Setting yaw velocity.. {heading}")

    if(heading > 0):
        dir = 1
    else:
        dir = -1
    heading = abs(heading)

    
    if relative:
        is_relative=1 #yaw relative to direction of travel
    else:
        is_relative=0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        0,    # param 1, yaw in degrees
        heading,          # param 2, yaw speed deg/s
        dir,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)
    print(f"[{Fore.GREEN}+{Style.RESET_ALL}] Yaw velocity set")

def connect_vehicle(port='tcp:127.0.0.1:5762'):    
    # Colored log message for connection
    print(f"[{Fore.LIGHTBLUE_EX}*{Style.RESET_ALL}] Connecting to vehicle on: {port}")
    vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True, timeout=60)
    print(f"[{Fore.GREEN}+{Style.RESET_ALL}] Vehicle connected on: {port}")
    return vehicle

def guide(vehicle: Vehicle):
    print(f"[{Fore.LIGHTBLUE_EX}*{Style.RESET_ALL}] Putting vehicle on guided mode..")
    vehicle.mode = VehicleMode("GUIDED")
    while not vehicle.mode.name == "GUIDED":
        print(f"{Fore.LIGHTBLUE_EX}    -> Vehicle Mode: {vehicle.mode.name}{Style.RESET_ALL}")
        time.sleep(1)
    print(f"{Fore.LIGHTBLUE_EX}    -> Vehicle Mode: {vehicle.mode.name}{Style.RESET_ALL}")
    print(f"[{Fore.GREEN}+{Style.RESET_ALL}] Vehicle now in guided mode")

def check_armable(vehicle: Vehicle):
    print(f"[{Fore.LIGHTBLUE_EX}*{Style.RESET_ALL}] Checking whether the vehicle is armable")
    while not vehicle.is_armable:
        print(f"{Fore.LIGHTBLUE_EX}    -> Armable State {vehicle.is_armable}{Style.RESET_ALL}")
        time.sleep(1)
    print(f"{Fore.LIGHTBLUE_EX}    -> Armable State {vehicle.is_armable}{Style.RESET_ALL}")
    print(f"[{Fore.GREEN}+{Style.RESET_ALL}]Vehicle is armable{Style.RESET_ALL}")

def arm(vehicle: Vehicle):
    print(f"[{Fore.LIGHTBLUE_EX}*{Style.RESET_ALL}] Arming the vehicle..")
    vehicle.armed = True
    while not vehicle.armed:
        print(f"{Fore.LIGHTBLUE_EX}    -> Vehicle armed status: {vehicle.armed}{Style.RESET_ALL}")
        time.sleep(1)
    print(f"{Fore.LIGHTBLUE_EX}    -> Vehicle armed status: {vehicle.armed}{Style.RESET_ALL}")
    print(f"[{Fore.GREEN}+{Style.RESET_ALL}] Vehicle armed")

def takeoff(vehicle: Vehicle, altitude):
    print(f"[{Fore.LIGHTBLUE_EX}*{Style.RESET_ALL}] Taking off..")
    vehicle.simple_takeoff(altitude)
    for i in range(0, 20):
        print(f"{Fore.LIGHTBLUE_EX}    -> Vehicle altitude: {vehicle.location.global_relative_frame.alt}{Style.RESET_ALL}")
        if vehicle.location.global_relative_frame.alt >= altitude*0.90:
            print(f"{Fore.LIGHTBLUE_EX}    -> Target altitude reached{Style.RESET_ALL}")
            break
        time.sleep(1)
    print(f"{Fore.LIGHTBLUE_EX}    -> Vehicle altitude: {vehicle.location.global_relative_frame.alt}{Style.RESET_ALL}")
    print(f"[{Fore.GREEN}+{Style.RESET_ALL}] Takeoff complete")

def land(vehicle: Vehicle):
    print(f"[{Fore.LIGHTBLUE_EX}*{Style.RESET_ALL}] Landing the vehicle..")
    vehicle.mode = VehicleMode("LAND")
    while not vehicle.mode.name == "LAND":
        print(f"{Fore.LIGHTBLUE_EX}    -> Vehicle Mode: {vehicle.mode.name}{Style.RESET_ALL}")
        time.sleep(1)
    print(f"{Fore.LIGHTBLUE_EX}    -> Vehicle Mode: {vehicle.mode.name}{Style.RESET_ALL}")
    # print(f"[{Fore.GREEN}+{Style.RESET_ALL}] Vehicle now in land mode")

    while True:
        print(f"{Fore.LIGHTBLUE_EX}    -> Vehicle altitude: {vehicle.location.global_relative_frame.alt}{Style.RESET_ALL}")
        if vehicle.location.global_relative_frame.alt <= 0.1:
            print(f"[{Fore.GREEN}+{Style.RESET_ALL}]Landing complete")
            break
        time.sleep(1)

forward_velocity = 0

def keyboard_control(vehicle: Vehicle, key:str):
    global forward_velocity
    if key == "w":
        forward_velocity += 0.1
        # set_velocity_twards_yaw_heading(vehicle, 3)
    if key == "a":
        yaw_heading = vehicle.heading - 10
        if( yaw_heading < 0):
            yaw_heading += 360
        set_yaw_angle(vehicle, yaw_heading)
    if key == "d":
        yaw_heading = vehicle.heading + 10
        if( yaw_heading > 360):
            yaw_heading -= 360
        set_yaw_angle(vehicle, yaw_heading)
    if key == "s":
        forward_velocity -= 0.5
        if forward_velocity < 0:
            forward_velocity = 0.001
    set_velocity_twards_yaw_heading(vehicle, forward_velocity)

if __name__ == "__main__":

    vehicle = connect_vehicle()
    log_stats(vehicle)
    time.sleep(5)
    guide(vehicle)
    check_armable(vehicle)
    arm(vehicle)
    takeoff(vehicle, 6)
    time.sleep(5)

    # Flight Code Here-----------------------
    while True:
        # an opencv window
        cv2.namedWindow("Drone Control")
        # resize the window
        cv2.resizeWindow("Drone Control", 300, 300)

        # create a black image
        img = np.zeros((300, 300, 3), np.uint8)

        # Attach mouse event
        # cv2.setMouseCallback("Drone Control", keyboard_control, vehicle)

        # show the image
        cv2.imshow("Drone Control", img)

        # wait for a key press (1 ms)
        key = cv2.waitKey(1) & 0xFF

        keyboard_control(vehicle, chr(key))

        # if the 'q' key is pressed, break from the loop
        if key == ord("q"):
            break
    
    # destroy all windows
    cv2.destroyAllWindows()

    # Check for key press event


    # ----------------------------------------

    log_stats(vehicle)
    time.sleep(5)
    land(vehicle)
