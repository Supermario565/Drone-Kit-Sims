# Drone-Kit-Sims
Collection of our Drone-Kit Simulations for Experimentation

## To Run
on Dmitri Lab PC:
Start Docker container holding DroneKit SITL (https://github.com/dronekit/dronekit-sitl.git) Framework. 

```
sudo docker run -it --privileged  --env=LOCAL_USER_ID="$(id -u)"  -v /tmp/.X11-unix:/tmp/.X11-unix:ro  -e DISPLAY=:0  -p 14556:14556/udp  -p 8080:8080  evans000/uav-sitl bash
```
Change to Home directory
```
cd /home
```
To run a SITL example simulation: 
```
git clone https://github.com/dronekit/dronekit-python.git
cd dronekit-python/examples
cd /<some_example>
python <example_proj>.py
```
Currently the following sims are known to work as is:
* Channel Overrides
* Create Attribute
* Flight Replay
* Vehicle State
* Simple GoTo
* Set Altitude Target
* MicroGCS
* Guided Set Speed Yaw
* 

### Sample goto Simulation
```
root@b7327068e335:/home/dronekit-python/examples/simple_goto# python simple_goto.py
```

You should get the following:
```
Starting copter simulator (SITL)
Downloading SITL from http://dronekit-assets.s3.amazonaws.com/sitl/copter/sitl-linux-copter-3.3.tar.gz
Download Complete.
Payload Extracted.
Ready to boot.
Connecting to vehicle on: tcp:127.0.0.1:5760
>>> APM:Copter V3.3 (d6053245)
>>> Frame: QUAD
>>> Calibrating barometer
>>> Initialising APM...
>>> barometer calibration complete
>>> GROUND START
Basic pre-arm checks
 Waiting for vehicle to initialise...
 Waiting for vehicle to initialise...
 Waiting for vehicle to initialise...
 Waiting for vehicle to initialise...
 Waiting for vehicle to initialise...
Arming motors
 Waiting for arming...
 Waiting for arming...
 Waiting for arming...
 Waiting for arming...
>>> ARMING MOTORS
>>> GROUND START
 Waiting for arming...
>>> Link timeout, no heartbeat in last 5 seconds
>>> ...link restored.
>>> Initialising APM...
Taking off!
 Altitude:  0.0
 Altitude:  0.0
 Altitude:  0.34
 Altitude:  1.73
 Altitude:  3.63
 Altitude:  6.25
 Altitude:  8.23
 Altitude:  9.38
 Altitude:  9.74
Reached target altitude
Set default/target airspeed to 3
Going towards first point for 30 seconds ...
Going towards second point for 30 seconds (groundspeed set to 10 m/s) ...
Returning to Launch
Close vehicle object
```

