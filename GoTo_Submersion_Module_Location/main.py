"""
Author: Dmitri Lyalikov

This script connects to the Copter and polls for armability and armed status.
Run this simulation in PC Docker Environment. as python main.py
Careful, we are using python 2.7 here.

"""

from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative

import argparse
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect', 
                     help="Vehicle connection target string. If not specified, SITL automatically started and used.")

args = parser.parse_args()
connection_string = args.connect
sitl = None 

# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """
    print("Basic pre-arm checks")

    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    # Take off and fly the vehicle to a specified altitude (meters)
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        #Break and return from function just below target altitude.        
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: 
            print("Reached target altitude")
            break
        time.sleep(1)

arm_and_takeoff(10)

print("Set default/target airspeed to 3")
# Airspeed in meters/second
vehicle.airspeed = 3

print("Going towards first point for 30 seconds ...")
point1 = LocationGlobalRelative(47.397742, 8.545594, 10)
vehicle.simple_goto(point1)

# sleep so we can see the change in map
time.sleep(30)

print("Going towards second point for 30 seconds (groundspeed set to 10 m/s) ...")
# Global location object: altitude relative to home location altitude
# Relative to WGS84, latitude, longitude, and altitude in meters
point2 = LocationGlobalRelative(47.397742, 8.545594, 10)

vehicle.simple_goto(point2, groundspeed=10)

# sleep so we can see the change in map
time.sleep(30)

print("Returning to Launch")  
"""
RTL Mode navigates the vehicle from its current position to hover above the home position.
The behavior of the mode is determined by the vehicle's home_location and the RTL_ALT parameter.

THe copter will rise a minimum of RLT_CLIMB_MIN meters or to RTL_ALT, whichever is higher, before returning home.
The default value for for RTL_ALT is 15 meters.

This means if you execute an RTL in Copter, it will return to the location where it was armed.
""" 
vehicle.mode = VehicleMode("RTL")
