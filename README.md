# spot_autonomy

UT Spot Autonomy Stack

## Dependencies
1. [ROS](http://wiki.ros.org/ROS/Installation)
1. System dependencies
    ```
    sudo apt install qt5-default libqt5websockets5-dev libgoogle-glog-dev libgflags-dev
    ```
1. [AMRL Fork of Clearpath Spot ROS Interface](https://github.com/ut-amrl/spot_ros), forked from https://github.com/clearpathrobotics/spot_ros

## Build
1. Clone and add repo paths to `ROS_PACKAGE_PATH`
    1.  [amrl_msgs](https://github.com/ut-amrl/amrl_msgs)
    1.  spot_autonomy 
1. Run `make`
1. Add commands to do step 1. to bashrc (otherwise need to repeat this step for launching spot_autonomy in every new shell session).

## External Hardware
* See https://github.com/ut-amrl/amrl-documentation/blob/master/robots/spot.md


## Usage

1. **Initial Setup Only:** Replicate `launch/start_clearpath_spot.launch.example` to `launch/start_clearpath_spot.launch`, filling in your Spot robot username, password, and IP address.
2. [Optional, if running under docker]:
    ```
    cd /home/amrl-user/ros-docker
    docker-compose up
    ```
    Open an interactive shell to the docker instance (run any ros/spot modules here):
    ```
    docker exec -it ros-docker_app_1 bash
    ```
4. `roslaunch spot_autonomy start_all.launch`

## Joystick

### Spektrum DXS SPMR1010

![Spektrum DXS SPMR1010](https://user-images.githubusercontent.com/35668665/200152606-df0bd43d-4079-43bd-828d-2f3af82a2f95.jpg)

* D Switch: Manual and Autonomous Control
  * "0" (away): Autonomous Control
  * "1" (neutral): Stop Robot
  * "2" (back): Manual Control
* A Button: Start/Stop `rosbag record`

While in Manual Control:

* Left Joystick: Yaw Control
* Right Joystick: Translation Control
* B Switch: Sit and Stand
  * "0" (down): Sit
  * "1" (neutral): Resume Manual Control
  * "2" (up): Stand
* F Switch: Speed Control
  * "0" (away): Full Speed
  * "1" (back): ~70% Speed (we cannot customize this)

In addition to the "F" switch, the controller has a set of four "Trim" sliders
that change the zero point of the joysticks (and thus their maximum and minimum
values). These sliders emit audible chirps. A higher-pitched chirp is emitted
when the neutral zero point is set.

### PS4 Dualshock Controller

![PS4 Dualshock Controller](https://upload.wikimedia.org/wikipedia/commons/thumb/5/59/DualShock_4.jpg/271px-DualShock_4.jpg)

* Left bumper: Press and hold for manual control
* Right bumper: Press to enable autonomous mode, press any other button to go out of autonomous mode
* Left joystick + LB: Yaw control
* Right joystick + LB: Translation control
* X button + LB: Stand up
* Square button + LB: Sit down
* Triangle button + LB: Start rosbag record
* Circle button + LB: Stop rosbag record

The rosbag record command is defined in `config/joystick.lua`.
