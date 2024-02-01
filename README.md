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

