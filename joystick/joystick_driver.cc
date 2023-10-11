// Copyright 2017 slane@cs.umass.edu
// College of Information and Computer Sciences,
// University of Massachusetts Amherst
//
//========================================================================
// This software is free: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License Version 3,
// as published by the Free Software Foundation.
//
// This software is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// Version 3 in the file COPYING that came with this distribution.
// If not, see <http://www.gnu.org/licenses/>.
//========================================================================

// Joystick Driver main file

#include <stdint.h>
#include <stdlib.h>

#include <iostream>
#include <numeric>
#include <string>
#include <vector>

#include "config_reader/config_reader.h"
#include "geometry_msgs/Twist.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "joystick/joystick.h"
#include "math/math_util.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Bool.h"
#include "std_srvs/SetBool.h"
#include "std_srvs/Trigger.h"
#include "util/timer.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"

DECLARE_int32(v);
DEFINE_int32(idx, 0, "Joystick index");
DEFINE_double(max_cmd_age, 0.1, "Maximum permissible age of autonomous command");

DEFINE_string(config, "config/joystick.lua", "Config file");

CONFIG_FLOAT(axis_scale, "Mapping.axis_scale");
CONFIG_INT(left_horiz_axis, "Mapping.left_horiz_axis");
CONFIG_INT(left_vert_axis, "Mapping.left_vert_axis");
CONFIG_INT(right_horiz_axis, "Mapping.right_horiz_axis");
CONFIG_INT(right_vert_axis, "Mapping.right_vert_axis");
CONFIG_INT(left_protruding_front_toggle_axis, "Mapping.left_protruding_front_toggle_axis");
CONFIG_INT(left_black_button_axis, "Mapping.left_black_button_axis");
CONFIG_INT(left_protruding_up_toggle_axis, "Mapping.left_protruding_up_toggle_axis");
CONFIG_INT(left_black_button, "Mapping.left_black_button");

CONFIG_STRING(rosbag_record_cmd, "record_cmd");

using joystick::Joystick;
using sensor_msgs::Joy;
using std::string;
using std::vector;

enum class JoystickState { STOPPED = 0, MANUAL = 1, AUTONOMOUS = 2 };

JoystickState state_ = JoystickState::STOPPED;
double t_last_cmd_ = 0;
geometry_msgs::Twist last_cmd_;
geometry_msgs::Twist manual_cmd_;
ros::Publisher cmd_publisher_;
ros::Publisher enable_autonomy_publisher_;
ros::ServiceClient sit_service_;
ros::ServiceClient stand_service_;
ros::Publisher left_black_button_value_publisher_;
ros::Publisher left_protruding_up_toggle_axis_value_publisher_;
ros::Publisher left_vert_axis_value_publisher_;
bool sitting_ = false;
int32_t demo_mode_ = 0;

geometry_msgs::Twist ZeroTwist() {
  geometry_msgs::Twist msg;
  msg.angular.x = 0;
  msg.angular.y = 0;
  msg.angular.z = 0;
  msg.linear.x = 0;
  msg.linear.y = 0;
  msg.linear.z = 0;
  return msg;
}

void SwitchState(const JoystickState& s) {
  CHECK_GE(static_cast<int>(s), 0);
  CHECK_LE(static_cast<int>(s), 2);
  static const std::vector<std::string> states = {"Stopped", "Manual", "Autonomous"};
  const std::string state_str = states[static_cast<int>(s)];
  if (FLAGS_v > 0) printf("Switch to %s\n", state_str.c_str());
  state_ = s;
  Sleep(0.5);
}

void UpdateState(const vector<int32_t>& buttons, const vector<float>& axes) {
  if (demo_mode_ == 2) {
    if (state_ != JoystickState::MANUAL) {
      SwitchState(JoystickState::MANUAL);
    }
    return;
  }
  switch (state_) {
    case JoystickState::STOPPED: {
      if (axes[CONFIG_left_protruding_up_toggle_axis] < 0) {
        SwitchState(JoystickState::MANUAL);
      } else if (axes[CONFIG_left_protruding_up_toggle_axis] > 0) {
        SwitchState(JoystickState::AUTONOMOUS);
      }
    } break;
    case JoystickState::MANUAL: {
      if (axes[CONFIG_left_protruding_up_toggle_axis] >= 0) {
        SwitchState(JoystickState::STOPPED);
      }
    } break;
    case JoystickState::AUTONOMOUS: {
      if (axes[CONFIG_left_protruding_up_toggle_axis] <= 0) {
        SwitchState(JoystickState::STOPPED);
      }
    } break;
    default: {
      // Must never happen.
      fprintf(stderr, "ERROR: Unknown joystick state %d\n", static_cast<int>(state_));
      exit(1);
    }
  }
}

void CommandCallback(const geometry_msgs::Twist& msg) {
  t_last_cmd_ = GetMonotonicTime();
  last_cmd_ = msg;
}

void DemoModeCallback(const std_msgs::Int32& msg) { demo_mode_ = msg.data; }

void PublishCommand() {
  // Don't publish drive commands when sitting!
  if (sitting_) return;
  if (state_ == JoystickState::MANUAL) {
    cmd_publisher_.publish(manual_cmd_);
  } else if (state_ == JoystickState::AUTONOMOUS) {
    const double t = GetMonotonicTime();
    if (t > t_last_cmd_ + FLAGS_max_cmd_age) {
      last_cmd_ = ZeroTwist();
    }
    cmd_publisher_.publish(last_cmd_);
  } else {
    cmd_publisher_.publish(ZeroTwist());
  }
}

float JoystickValue(float x, float scale) {
  static const float kDeadZone = 0.02;
  if (fabs(x) < kDeadZone) return 0;
  return ((x - math_util::Sign(x) * kDeadZone) / (1.0f - kDeadZone) * scale);
}

void SetManualCommand(const vector<int32_t>& buttons, const vector<float>& axes) {
  const float kMaxLinearSpeed = 1.6;
  const float kMaxRotationSpeed = math_util::DegToRad(90);
  std_srvs::Trigger trigger_req;
  manual_cmd_.linear.x = JoystickValue(axes[CONFIG_right_vert_axis], CONFIG_axis_scale * kMaxLinearSpeed);
  manual_cmd_.linear.y = JoystickValue(axes[CONFIG_right_horiz_axis], CONFIG_axis_scale * kMaxLinearSpeed);
  manual_cmd_.angular.z = JoystickValue(axes[CONFIG_left_horiz_axis], CONFIG_axis_scale * kMaxRotationSpeed);

  if (state_ == JoystickState::MANUAL) {
    bool do_sit = false;
    bool do_stand = false;
    do_sit = axes[CONFIG_left_protruding_front_toggle_axis] > 0;
    do_stand = axes[CONFIG_left_protruding_front_toggle_axis] < 0;

    if (do_sit) {
      if (!sit_service_.call(trigger_req)) {
        fprintf(stderr, "Error calling sit service!\n");
      } else {
        sitting_ = true;
      }
      Sleep(0.5);
    } else if (do_stand) {
      if (!stand_service_.call(trigger_req)) {
        fprintf(stderr, "Error calling stand service!\n");
      } else {
        sitting_ = false;
      }
      Sleep(0.5);
    }
  }
}

void LoggingControls(const vector<int32_t>& buttons) {
  // NOT active, its commented out below!
  // See if recording should start.
  if ((int)buttons.size() >= CONFIG_left_black_button && buttons[CONFIG_left_black_button] == 1) {
    static bool recording = false;
    if (recording && buttons[CONFIG_left_black_button] == 1) {
      recording = false;
      if (system("rosnode kill joystick_rosbag_record") != 0) {
        printf("Unable to kill rosbag!\n");
      } else {
        printf("Stopped recording rosbag.\n");
      }
      Sleep(0.5);
    } else if (!recording && buttons[CONFIG_left_black_button] == 1) {
      printf("Starting recording rosbag...\n");
      if (system(CONFIG_rosbag_record_cmd.c_str()) != 0) {
        printf("Unable to record\n");
      } else {
        printf("Started recording rosbag.\n");
        recording = true;
      }
      Sleep(0.5);
    }
  }
}

std_msgs::Float32 GetLeftProtrudingUpToggleAxisMsg(const vector<int32_t>& buttons, const vector<float>& axes) {
  std_msgs::Float32 msg;
  msg.data = axes[CONFIG_left_protruding_up_toggle_axis];
  return msg;
}

std_msgs::Float32 GetLeftVertAxisMsg(const vector<int32_t>& buttons, const vector<float>& axes) {
  std_msgs::Float32 msg;
  msg.data = axes[CONFIG_left_vert_axis];
  return msg;
}

std_msgs::Int32 GetLeftBlackButtonMsg(const vector<int32_t>& buttons) {
  std_msgs::Int32 msg;
  msg.data = buttons[CONFIG_left_black_button];
  return msg;
}

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InitGoogleLogging(argv[0]);
  ros::init(argc, argv, "joystick");
  config_reader::ConfigReader reader({FLAGS_config});
  // printf("%s\n", CONFIG_rosbag_record_cmd.c_str());
  ros::NodeHandle n;
  ros::Publisher publisher = n.advertise<sensor_msgs::Joy>("joystick", 1);
  ros::Subscriber cmd_subscriber = n.subscribe("navigation/cmd_vel", 10, CommandCallback);
  ros::Subscriber demo_mode_subscriber = n.subscribe("/demo_mode_num", 10, DemoModeCallback);
  cmd_publisher_ = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  stand_service_ = n.serviceClient<std_srvs::Trigger>("spot/stand");
  sit_service_ = n.serviceClient<std_srvs::Trigger>("spot/sit");
  enable_autonomy_publisher_ = n.advertise<std_msgs::Bool>("autonomy_arbiter/enabled", 1);
  left_black_button_value_publisher_ = n.advertise<std_msgs::Int32>("left_black_button_value", 1);
  left_protruding_up_toggle_axis_value_publisher_ = n.advertise<std_msgs::Float32>("left_protruding_up_toggle_axis_value", 1);
  left_vert_axis_value_publisher_ = n.advertise<std_msgs::Float32>("left_vert_axis_value", 1);
  Joystick joystick;
  if (!joystick.Open(FLAGS_idx)) {
    fprintf(stderr, "ERROR: Unable to open joystick)!\n");
    return (1);
  }

  vector<int32_t> buttons;
  vector<float> axes;
  Joy msg;
  msg.header.frame_id = "joystick";
  msg.header.seq = 0;

  manual_cmd_ = ZeroTwist();
  RateLoop rate_loop(60);
  std_msgs::Bool enable_autonomy_msg;
  enable_autonomy_msg.data = false;
  while (ros::ok()) {
    joystick.ProcessEvents(2);
    joystick.GetAllAxes(&axes);
    joystick.GetAllButtons(&buttons);
    UpdateState(buttons, axes);
    SetManualCommand(buttons, axes);
    PublishCommand();
    // LoggingControls(buttons);
    msg.header.stamp = ros::Time::now();
    msg.axes = axes;
    msg.buttons = buttons;
    publisher.publish(msg);
    if (state_ == JoystickState::AUTONOMOUS) {
      enable_autonomy_msg.data = true;
    } else {
      enable_autonomy_msg.data = false;
    }
    enable_autonomy_publisher_.publish(enable_autonomy_msg);
    left_black_button_value_publisher_.publish(GetLeftBlackButtonMsg(buttons));
    left_protruding_up_toggle_axis_value_publisher_.publish(GetLeftProtrudingUpToggleAxisMsg(buttons, axes));
    left_vert_axis_value_publisher_.publish(GetLeftVertAxisMsg(buttons, axes));
    ros::spinOnce();
    rate_loop.Sleep();
  }

  joystick.Close();
  return 0;
}
