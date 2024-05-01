import time
import socket
import math
from dronekit import connect, VehicleMode,LocationGlobalRelative,APIException



def arm_and_takeoff(targetHeight):


    while vehicle.is_armable != True:
        print("Waiting for vehicle to become armable")
        time.sleep(1)
    print("Vehicle is armable")

    vehicle.mode = VehicleMode("GUIDED")

    while vehicle.mode != "GUIDED":
        print("Waiting for drone to enter GUIDED flight mode")
        time.sleep(1)
    print("Vehicle now in GUIDED MODE.")

    vehicle.armed = True
    while vehicle.armed == False:
        print("Waiting for vehicle to become armed.")
        time.sleep(1)

    print("Propellers Should be spinning")

    vehicle.simple_takeoff(targetHeight)  # In meters

    while True:
        print(f"Current Altitude: {vehicle.location.global_relative_frame.alt}")
        if vehicle.location.global_relative_frame.alt >= targetHeight * .95:
            break
        time.sleep(1)
    print("Target altitude reached!!")
    return None


vehicle = connect('/dev/ttyACM0', wait_ready=True, baud=57600)

arm_and_takeoff(5)