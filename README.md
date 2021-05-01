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

## Usage With Physical Spot

1. Replicate `launch/start_all_example.launch` to `launch/start_all.launch`, filling in your Spot robot username, password, and IP address.
1. `roslaunch spot_autonomy start_all.launch`

## Joystick

* Left bumper: Press and hold for manual control
* Right number: Press to enable autonomous mode, press any other button to go out of autonomus mode
* Left joystick: Yaw control when in manual mode
* Right joystick + LB: Translation control
* Y(Yellow) button + LB: Stand up
* A(Green) button + LB: Sit down
* X(Blue) button + LB: Start rosbag record
* B(Red) button + LB: Stop resbag record

## Usage With Spot Simulation

To run spot_autonomy with the [Spot simulation](https://github.com/utexas-bwi/spot)

1. Replicate `launch/start_spot_interface.launch.example` to  `launch/start_spot_interface.launch`, changing the IP address to 127.0.0.1 and adding `--sim` to the end of the `args=` line.
2. Copy `config/config.hpp` to `robofleet_client/src`
3. Copy `config/graph_navigation/navigation.sim.lua` to `graph_navigation/config/`
4. Copy `config/websocket_main.cc` to `webviz/src/websocket`

## Running with Spot Simulation
Run:
* The Spot simulation server: `rosrun spot_comm spotServer` from a the top level directory of the catkin workspace containing the `utexas-bwi/spot` repository
* The Spot simulation Gazebo world: `roslaunch spot_gazebo spot_ahg_nav.launch`
* `roslaunch spot_autonomy start_all.launch simulation:=true` in a Python 3.7 virtual environment
* `./bin/websocket` from the top level directory

## Visualize the Simulated Spot's Behavior
* Open `webviz/webviz.html` in a browser
* Click the connect button
* Set the Spot's pose
* Set a navigation target
