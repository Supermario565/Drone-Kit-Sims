# Load/Start Auto Mission
## Basic Mission Operations
* Downloading missions from the vehicle
* Clearing missions
* Creating mission commands and uploading to the vehicle
* Monitoring the current active command
* Changing the current active command

## Example Mission
```Python
add_square_mission(vehicle.location.global_frame, 50)
```
Clears the crrent mission, and defines a new mission with a takeoff command and four waypoints arranged in a square around the central position. We use `next` to determine when we have reached the final point. 

After taking off in guided mode, start the mission by setting the mode to `AUTO`

```Python
vehicle.mode = VehicleMode("AUTO")
```

Convenience function `distance_to_current_waypoint()` gets the distrance to the next waypoint and 