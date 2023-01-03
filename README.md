# ROS Noetic container

includes mavros
includes custom offboard package
includes ZED SDK

```
sudo docker build . \ 	
  --build-arg L4T_MAJOR_VERSION=35 \
  --build-arg L4T_MINOR_VERSION=1 \
  --build-arg L4T_PATCH_VERSION=0 \
  --build-arg ZED_SDK_MAJOR=3 \
  --build-arg ZED_SDK_MINOR=8 \
  -t duanenielsen/ros:noetic-ros-offboard-l4t-r35.1.0
```

to run 

```
sudo docker run --runtime nvidia -it --rm --device=/dev/ttyACM0 --privileged --gpus all --network host duanenielsen/ros:noetic-ros-offboard-l4t-r35.1.0
```


handy bash functions

```
dockerbuild () {
  local command="sudo docker build ."
  local version="--build-arg L4T_MAJOR_VERSION=35
                 --build-arg L4T_MINOR_VERSION=1   
                 --build-arg L4T_PATCH_VERSION=0   
                 --build-arg ZED_SDK_MAJOR=3   
                 --build-arg ZED_SDK_MINOR=8   
                 --build-arg JETPACK_MAJOR=5   
                 --build-arg JETPACK_MINOR=0.2   
                 --build-arg L4T_BASE_IMAGE=l4t-jetpack"
  $command $version "$@"
}

dockerrun () {
  sudo xhost +si:localuser:root
  XAUTH=/tmp/.docker.xauth
  xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
  chmod 777 $XAUTH
  local command="sudo docker run"
  local options="--runtime nvidia
                 -it
                 --rm
                 --device=/dev/ttyACM0
                 --network host
                 -e ROS_LOG_LEVEL=debug
                 --gpus all
                 -e DISPLAY=$DISPLAY
                 -v /tmp/.X11-unix:/tmp/.X11-unix
                 -e XAUTHORITY=$XAUTH
                 -v $XAUTH:$XAUTH
                 --privileged"
  local image=$1
  $command $options $image
}

```
