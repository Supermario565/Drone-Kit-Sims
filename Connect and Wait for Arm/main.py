"""
Author: Dmitri Lyalikov 

This script connects to the Copter and polls for armability and armed status.
Run this simulation in PC Docker Environment. as python connect-arm.py
Careful, we are using python 2.7 here.
"""

import dronekit_sitl
sitl = dronekit_sitl.start_default()
connection_string = sitl.connection_string()

from dronekit import connect, VehicleMode

print "Connecting to vehicle on: %s" % connection_string
vehicle = connect(connection_string, wait_ready=True)

# Wait for the vehicle to be armable
while not vehicle.is_armable:
    print " Waiting for vehicle to initialise..."
    time.sleep(.5)

print "Armed: %s" % vehicle.armed

print "Get some vehicle attribute values:"
print " GPS: %s" % vehicle.gps_0
print " Battery: %s" % vehicle.battery
print " Is Armable?: %s" % vehicle.is_armable
print " System status: %s" % vehicle.system_status.state
print " Mode: %s" % vehicle.mode.name    # settable
print " Groundspeed: %s" % vehicle.groundspeed
print " Heading: %s" % vehicle.heading
print " Attitude: %s" % vehicle.attitude
print " GPS: %s" % vehicle.gps_0
print " Last Heartbeat: %s" % vehicle.last_heartbeat


# Close vehicle object before exiting script
vehicle.close()

# Shut down simulator
sitl.stop()
