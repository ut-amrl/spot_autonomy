# spot_autonomy

UT Spot Autonomy Stack

## Dependencies
1. [ROS](http://wiki.ros.org/ROS/Installation)
1. System dependencies
  ```
  sudo apt install qt5-default libqt5websockets5-dev libgoogle-glog-dev libgflags-dev
  ```
1. [spot-ros-wrapper](https://github.com/ut-amrl/spot-ros-wrapper)

## Build
1. Add the repo path to `ROS_PACKAGE_PATH`
1. Run `make`

## Usage

1. Replicate `launch/start_all_example.launch` to `launch/start_all.launch`, filling in your Spot robot username, password, and IP address.
1. `roslaunch spot_autonomy start_all.launch`

