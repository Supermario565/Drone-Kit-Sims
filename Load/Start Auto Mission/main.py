

#Creating a pre-programmed circle mission

from __future__ import print_function
import math
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative

import argparse

def download_missions():
    """
    Download missions from the vehicle.
    """
    print("Downloading missions...")
    missions = vehicle.commands
    missions.download()
    missions.wait_ready()
    print("Missions downloaded successfully.")

download_missions()

parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect', 
                     help="Vehicle connection target string. If not specified, SITL automatically started and used.")

args = parser.parse_args()
connection_string = args.connect
sitl = None

# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    print("Starting copter simulator (SITL)")

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
vehicle.airspeed = 3


#Sending Drone to a location
# Define the center of the circle
center_lat = 37.8736
center_lon = -122.254

# Define the radius of the circle
radius = 10  # in meters

# Define the duration of the circle flight
circle_duration = 180  # in seconds

# Calculate the number of waypoints to create a circle
num_waypoints = int(circle_duration / 2)  # Assuming 2 seconds per waypoint

# Calculate the angle between each waypoint
angle_increment = 2 * math.pi / num_waypoints

# Start the circle flight
print("Starting circle flight ...")
for i in range(num_waypoints):
    # Calculate the coordinates of the current waypoint on the circle
    angle = i * angle_increment
    waypoint_lat = center_lat + radius * math.cos(angle)
    waypoint_lon = center_lon + radius * math.sin(angle)
    waypoint = LocationGlobalRelative(waypoint_lat, waypoint_lon, 10)
    
    # Go to the current waypoint
    vehicle.simple_goto(waypoint)
    
    # Sleep for 2 seconds before going to the next waypoint
    time.sleep(2)

# Return to launch
print("Returning to Launch")
vehicle.mode = VehicleMode("RTL")

# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

# Shut down simulator
if sitl:
    sitl.stop()

print("Completed")

