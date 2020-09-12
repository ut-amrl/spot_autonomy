//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    pointcloud_to_laserscan_main.cc
\brief   A point cloud to laserscan convert that actually works without
         crashing or throwing buffer overflow errors.
\author  Joydeep Biswas, (C) 2020
*/
//========================================================================

#include <signal.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <vector>

#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "glog/logging.h"
#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "ros/ros.h"
#include "math/math_util.h"
#include "util/timer.h"
#include "ros/ros_helpers.h"
#include "pcl_ros/conversions.h"

using math_util::DegToRad;
using math_util::RadToDeg;
using ros::Time;
using ros_helpers::Eigen3DToRosPoint;
using ros_helpers::Eigen2DToRosPoint;
using ros_helpers::RosPoint;
using ros_helpers::SetRosVector;
using sensor_msgs::LaserScan;
using sensor_msgs::PointCloud2;
using std::string;
using std::vector;
using Eigen::Vector2f;

DEFINE_string(laser_topic, 
    "velodyne_2dscan", "Name of ROS topic for LIDAR data");
DEFINE_string(pointcloud_topic, 
    "velodyne_points", "Name of ROS topic for point cloud data");

DECLARE_string(helpon);
DECLARE_int32(v);

sensor_msgs::LaserScan last_laser_msg_;
bool run_ = true;

void SignalHandler(int) {
  if (!run_) {
    printf("Force Exit.\n");
    exit(0);
  }
  printf("Exiting.\n");
  run_ = false;
}

void PointcloudCallback(const sensor_msgs::PointCloud2& msg) {
   pcl::PointCloud<pcl::PointXYZ> cloud;
   pcl::fromROSMsg(msg, cloud);
}

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InitGoogleLogging(argv[0]);
  signal(SIGINT, SignalHandler);
  // Initialize ROS.
  ros::init(argc, argv, "pointcloud_to_laserscan");
  ros::NodeHandle n;

  ros::Subscriber pointcloud_sub =
      n.subscribe(FLAGS_pointcloud_topic, 1, &PointcloudCallback);

  ros::spin();
  return 0;
}
