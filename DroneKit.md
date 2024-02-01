[Introducing DroneKit-Python](https://dronekit-python.readthedocs.io/en/latest/about/index.html) Runs on-board companion computer (In our case DCC/Jetson Nano) and communicate with the Ardupilot flight controller using a low latency link (In our case UART). Can also be used for ground station apps, with a higher latency RF-link 

API communicates via MAVLink. Provides:
* Telemetry
* State
* Parameter information
* Mission management
* Direct control over vehicle movement and operations
## In This Document
* API
* Installation
	* Hello Drone Example
* Simulated Vehicles
	* Connecting to an Additional Ground Station
* Connecting
	* Connecting Via Serial
* Launching
* Movement
	* Position Control
* Vehicle Info 
* Missions and Waypoints
* Exiting
* MAVLink Messages
	* Message Listener
* Limitations/Considerations
* Practices
* ETC

# API

* Connect to a vehicle
* Get and set state/telemetry and parameter information
* Receive asynchronous notification of state changes
* Guide a UAV to specified position (GUIDED mode)
* Send arbitrary custom messages to control UAV movement and other hardware
* Create and manage waypoint missions (AUTO mode)
* Override RC channel settings

# Installation 
```Python
sudo pip install dronekit
sudo pip isntall dronekit-sitl
```
## Hello Drone Example

Launch simulator, import and call connect method via TCP, return Vehicle object

```Python
import dronekit_sitl
sitl = dronekit_sitl.start_default()
connection_string = sitl_connection_string()

from dronekit import connect, VehicleMode

print("Connecting to vehicle on: %s" % (connection_string,))
vehicle = connect(connection_string, wait_ready=True)

print "GPS: %s" % vehicle.gps_0
print "Battery %s" % vehicle.battery
print "Last Heartbeat %s" % vehicle.last_heartbeat
print "Is Armable %s" % vehicle.is_armable
print "System status: %s" % vehicle.system_status.state
print "Mode: %s" % vehicle.mode.name

# Close vehicle connection before exiting script
vehicle.close()

# Shut down simulator 
sitl.stop()
print("Completed")
```

# Simulated Vehicles

SITL ([SITL (Software In The Loop)](http://dev.ardupilot.com/wiki/simulation-2/sitl-simulator-software-in-the-loop/)) allows creating and testing DroneKit apps without a vehicle. 

Run latest version of copter:
```Python
dronekit-sitl copter
```

Can specify a particular vehicle and version, and home location
```Python
dronekit-sitl plane-3.3.0 --home=35.363261,149.165230,584,353
```

## Connecting an Additional Ground Station
Can connect a ground station to an unused port to which messages are forwarded.
```Python
mavproxy.py --master tcp:127.0.0.1:5760 --sitl 127.0.0.1:5501 --out 127.0.0.1:14550 --out 127.0.0.1:14551
```

Once you have available ports, connect to ground station using one UDP address, and DroneKit-Python using other.

First connect script:
```Python
vehicle = connect('127.0.0.1:14550', wait_ready=True)
```

Then connect Mission Planner to the second UDP port:
*  [Download and install Mission Planner](http://ardupilot.com/downloads/?did=82)
* Ensure the selection list at top right says UDP and select `Connect` button next to it. Enter port number

# Connecting

```Python
from dronekit import connect

# Connect to the Vehicle (in this case a UDF endpoint)
vehicle = connect('REPLACE_connection_string_for_vehicle', wait_ready=True)
```


## For Serial UART (OUR CASE)

```Python
vehicle = connect('/dev/ttyAMA0', wait_ready=True, baud=57600)`
```

|Connection type|Connection string|
|---|---|
|Linux computer connected to the vehicle via USB|`/dev/ttyUSB0`|
|Linux computer connected to the vehicle via Serial port (RaspberryPi example)|`/dev/ttyAMA0` (also set `baud=57600`)|
|SITL connected to the vehicle via UDP|`127.0.0.1:14550`|
|SITL connected to the vehicle via TCP|`tcp:127.0.0.1:5760`|
|OSX computer connected to the vehicle via USB|`dev/cu.usbmodem1`|
|Windows computer connected to the vehicle via USB (in this case on COM14)|`com14`|
|Windows computer connected to the vehicle using a 3DR Telemetry Radio on COM14|`com14` (also set `baud=57600`)|

# Launching

Generally use standard launch sequence:
* Poll on `Vehicle.is_armable` until ready to arm
* Set the `Vehicle.mode` to `GUIDED`
* Set the `Vehicle.armed` to true and poll on same attribute until vehicle is armed
* Call the `Vehicle.simple_takeoff` with a target altitude
* Poll on altitude and allow the code to continue only when it is reached

```Python
vehicle = connect('dev/ttyAMA0', wait_ready=True)

def arm_and_takeoff(aTargetAltitude):
	"""
    Arms vehicle and fly to aTargetAltitude.
    """

    print "Basic pre-arm checks"
    # Don't try to arm until autopilot is ready (booted, EKF ready, GPS lock)
    while not vehicle.is_armable:
        print " Waiting for vehicle to initialise..."
        time.sleep(1)

    print "Arming motors"
    # Copter should arm in GUIDED mode
    vehicle.mode    = VehicleMode("GUIDED")
    vehicle.armed   = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print " Waiting for arming..."
        time.sleep(1)

    print "Taking off!"
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print " Altitude: ", vehicle.location.global_relative_frame.alt
        #Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
            print "Reached target altitude"
            break
        time.sleep(1)

arm_and_takeoff(20)
```


When this function returns the app can continue in GUIDED mode or switch to AUTO mode to start a mission.
## Movement
Can be controlled with:
* Explicitly setting a target position
* Specifying the vehicle's velocity components 

### Position Control
Useful when final position is fixed/known.
`Vehicle.simple_goto` for moving to a specific position (at a defined speed) 
Takes a `LocationGlobale` or `LocationGlobalRelative` argument

```Python
vehicle.mode = VehicleMode("GUIDED")

# Set global relative frame
a_location = LocationGlobalRelative(-34.36114, 149.166022, 30)
vehicle.simple_goto(a_location)

```


GUIDED mode is recommended for flying Copter autonomously without a predefined mission. 

`Vehicle.simple_goto` for moving to a specific position (at a defined speed) 

## Vehicle Info

Vehicle state information is exposed through vehicle _attributes_ which can be read and observed (and in some cases written) and vehicle settings which can be read, written, iterated and observed using _parameters_ (a special attribute). All the attributes are documented in [Vehicle State and Settings](https://dronekit-python.readthedocs.io/en/latest/guide/vehicle_state_and_parameters.html).

```Python
# vehicle is an instance of the Vehicle class
print "Autopilot Firmware version: %s" % vehicle.version
print "Autopilot capabilities (supports ftp): %s" % vehicle.capabilities.ftp
print "Global Location: %s" % vehicle.location.global_frame
print "Global Location (relative altitude): %s" % vehicle.location.global_relative_frame
print "Local Location: %s" % vehicle.location.local_frame    #NED
print "Attitude: %s" % vehicle.attitude
print "Velocity: %s" % vehicle.velocity
print "GPS: %s" % vehicle.gps_0
print "Groundspeed: %s" % vehicle.groundspeed
print "Airspeed: %s" % vehicle.airspeed
print "Gimbal status: %s" % vehicle.gimbal
print "Battery: %s" % vehicle.battery
print "EKF OK?: %s" % vehicle.ekf_ok
print "Last Heartbeat: %s" % vehicle.last_heartbeat
print "Rangefinder: %s" % vehicle.rangefinder
print "Rangefinder distance: %s" % vehicle.rangefinder.distance
print "Rangefinder voltage: %s" % vehicle.rangefinder.voltage
print "Heading: %s" % vehicle.heading
print "Is Armable?: %s" % vehicle.is_armable
print "System status: %s" % vehicle.system_status.state
print "Mode: %s" % vehicle.mode.name    # settable
print "Armed: %s" % vehicle.armed    # settable
```

See [Vehicle State and Settings (dronekit-python.readthedocs.io)](https://dronekit-python.readthedocs.io/en/latest/guide/vehicle_state_and_parameters.html) for further detail
## Missions and Waypoints

Can also [create and modify autonomous missions](https://dronekit-python.readthedocs.io/en/latest/guide/auto_mode.html#auto-mode-vehicle-control). 

AUTO mode is used to run pre-defined waypoint missions on copter, plane and rover.
Provides basic commands to:
* Download and clear the current mission commands from the vehicle
* Add and upload new mission commands
* Count number of waypoints
* Read and set the currently executed mission command

[Missions (AUTO Mode) (dronekit-python.readthedocs.io)](https://dronekit-python.readthedocs.io/en/latest/guide/auto_mode.html)
# Exiting

Call `Vehicle.close()` before exiting to ensure all messages have been flushed.

## MavLink Messages
How to intercept specific MAVLink messages by defining a listener callback function using the `Vehicle.on_message()` decorator

### Message Listener 

Can be used to set a particular function as the callback handler for a particular message type. 
```Python
# Create a message listener using the decorator
@vehicle.on_message('RANGEFINDER')
def listener(self, name, message):
    print message
```


# Limitations/Considerations
* Python 3 is not supported, requires Python 2.7
* Messages and message acknowledgements are not guaranteed to arrive (the protocol is not lossless)
* Commands may be silently ignored if Autopilot is not in a state where it can safely act on them
* Command ack and completion messages are not sent in most cases
* Commands may be interrupted before completion
* Commands can arrive at autopilot from multiple sources

## Practices
* Check that a vehicle is in a state to obey a command (poll on Vehicle.is_armable before trying to arm a vehicle)
* Do not assume a command has succeeded until the changed behavior is observed. 
* Monitor for state changes and react accordingly. For example if the user changes the mode from `GUIDED`, your script should stop sending commands



## ETC Useful Functions

```Python
def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the
    specified `original_location`. The returned LocationGlobal has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to
    the current vehicle position.

    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.

    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    if type(original_location) is LocationGlobal:
        targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
    else:
        raise Exception("Invalid Location object passed")

    return targetlocation;
```

```Python
def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two `LocationGlobal` or `LocationGlobalRelative` objects.

    This method is an approximation, and will not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

```


```Python
def get_bearing(aLocation1, aLocation2):
    """
    Returns the bearing between the two LocationGlobal objects passed as parameters.

    This method is an approximation, and may not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    off_x = aLocation2.lon - aLocation1.lon
    off_y = aLocation2.lat - aLocation1.lat
    bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
    if bearing < 0:
        bearing += 360.00
    return bearing;
```