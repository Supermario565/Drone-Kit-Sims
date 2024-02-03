"""
Author: Dmitri Lyalikov 

Simulates process of using YOLOv7 detections to approximate GPS location of detected objects.

Test 1 (No Detection):
    - Read prediction information from test_yolov7_output.txt 
    - Approximate Relative GPS location of detected objects using bounding box   
    - Use simple goto function to navigate to the detected object at some altitude

Test 2 (Detection):
    - Lodd YOLOv7 model
    - Run inference on test_yolov7_input.jpg
    - Approximate Relative GPS location of detected objects using bounding box   
    - Use simple goto function to navigate to the detected object at some altitude

"""

from __future__ import print_function
import time
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobalRelative

import argparse
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect', 
                     help="Vehicle connection target string. If not specified, SITL automatically started and used.")

args = parser.parse_args()
connection_string = args.connect
sitl = None
TARGET_ALTITUDE = 30
DETECT = False

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
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)


arm_and_takeoff(TARGET_ALTITUDE)

if DETECT:
    # Load YOLOv7 model
    # Run inference on test_yolov7_input.jpg
    # Approximate Relative GPS location of detected objects using bounding box
    # Use simple goto function to navigate to the detected object at some altitude
    pass
else:
    # Read prediction information from test_yolov7_output.txt
    # Approximate Relative GPS location of detected objects using bounding box
    # Use simple goto function to navigate to the detected object at some altitude

    pass


"""
TODO We need some logic in dealing with multiple interesting objects in the frame:
    Which do we pick as first target?
"""
def approximate_gps_location(objs, location, attitude, heading, pts_undist):
    """
    Approximate GPS location of detected objects using bounding box
    """
    # list of center points of all objects detected
    pts = []

    # Add the center point of each object to the list
    for obj in objs:
        classifier, probability, bbox = obj
        x, y, w, h = bbox 
        pts.append([x, y])

    if len(pts) > 0:
        # Generate a vector from the focal point to the camera plane for each detected object (x,y,z = right, back, down) relative to the UAV
        obj_vectors = np.hstack((pts_undist, np.ones((pts_undist.shape[0], 1))))


        # calculate the transformation matrix to orient points in the camera in a NED reference frame
        # Corrects for roll, pitch, and heading of the UAV
        mat_transform = matrix_rot_y(attitude.roll) @ matrix_rot_x(-attitude.pitch - CAM_MOUNT_ANGLE * pi / 180) @ matrix_rot_z(-(90 + heading) * pi / 180)
        # Coordinates in format: [[classifier, waypoint], ...]
        gps_coords = []
        for i, obj in enumerate(objs):
            classifier, probability, bbox = obj
            # TODO We need to skip any objects that are not of interesting class

            obj_vec = obj_vectors[i]

            # transform the vector toward the object to be in a North-East-Down reference frame relative to the UAV
            ned_vec = obj_vec @ mat_transform
            # approximate lattitude and longitude by extending the object's vector from the location and altitude of the UAV down to the ground
            obj_lat = location.lat + location.alt * (ned_vec[0] / ned_vec[2]) * DEGREES_PER_METER
            obj_lon = location.lon + location.alt * (ned_vec[1] / ned_vec[2]) * DEGREES_PER_METER

            # send the approximate GPS location to the UAV
            waypoint = LocationGlobalRelative(obj_lat, obj_lon, TARGET_ALTITUDE)
            gps_coords.append([classifier, waypoint])

    return gps_coords