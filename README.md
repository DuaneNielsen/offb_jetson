# ROS Noetic container

includes mavros
includes custom offboard package

```
sudo docker build . -t duanenielsen/ros:noetic-ros-offboard-l4t-r35.1.0
```

to run 

```
sudo docker run --runtime nvidia -it --rm --device=/dev/ttyACM0 --network host duanenielsen/ros:noetic-ros-offboard-l4t-r35.1.0
```
