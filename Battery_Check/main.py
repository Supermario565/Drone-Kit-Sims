import dronekit_sitl
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time

sitl = dronekit_sitl.start_default()
connection_string = sitl.connection_string()

from dronekit import connect, VehicleMode

print ("Connecting to vehicle on: %s") % connection_string
vehicle = connect(connection_string, wait_ready=True)

while not vehicle.is_armable:
    print (" Waiting for vehicle to initialise...")
    time.sleep(.5)

print ("Armed: %s") % vehicle.armed

print ("Get some vehicle attribute values:")
print (" GPS: %s") % vehicle.gps_0
print (" Battery: %s") % vehicle.battery
print (" Is Armable?: %s") % vehicle.is_armable
print (" System status: %s") % vehicle.system_status.state
print (" Mode: %s") % vehicle.mode.name    # settable
print (" Groundspeed: %s") % vehicle.groundspeed
print (" Heading: %s") % vehicle.heading
print (" Attitude: %s") % vehicle.attitude
print (" GPS: %s") % vehicle.gps_0
print (" Last Heartbeat: %s") % vehicle.last_heartbeat

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
time.sleep()

class battery_check:

#Check/Alarm for battery
    def check_battery_level():
        battery = vehicle.battery
        #print(f"Battery Level {battery}")
        print("Battery level: {}%".format(battery.level))
        if battery.level <= 20:
            print("Battery level low!")
            vehicle.mode = VehicleMode("RTL")
            return battery.level

# Print battery level every 10 seconds
    while True:
        check_battery_level()
        time.sleep(10)

# Close vehicle connection before exiting script
vehicle.close()

# Shut down simulator 
sitl.stop()
print("Completed")