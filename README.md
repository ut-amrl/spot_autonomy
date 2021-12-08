# spot_autonomy

UT Spot Autonomy Stack

## Dependencies
1. [ROS](http://wiki.ros.org/ROS/Installation)
1. System dependencies
    ```
    sudo apt install qt5-default libqt5websockets5-dev libgoogle-glog-dev libgflags-dev
    ```
1. [Clearpath Spot ROS Interface](https://github.com/clearpathrobotics/spot_ros)

## Build
1. Add the repo path to `ROS_PACKAGE_PATH`
1. Run `make`

## Usage

1. **Initial Setup Only:** Replicate `launch/start_clearpath_spot.launch.example` to `launch/start_clearpath_spot.launch`, filling in your Spot robot username, password, and IP address.
1. `roslaunch spot_autonomy start_all.launch`

## Joystick

* Left bumper: Press and hold for manual control
* Right bumber: Press to enable autonomous mode, press any other button to go out of autonomus mode
* Left joystick: Yaw control when in manual mode
* Right joystick + LB: Translation control
* Y(Yellow) button + LB: Stand up
* A(Green) button + LB: Sit down
* X(Blue) button + LB: Start rosbag record
* B(Red) button + LB: Stop resbag record

The rosbag record command is defined in `config/joystick.lua`.

