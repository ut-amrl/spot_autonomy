#!/usr/bin/env python
from std_msgs.msg import Int64, Int32
from geometry_msgs.msg import PoseStamped
from amrl_msgs.msg import NavStatusMsg
import argparse
import os
import rospy
import tf.transformations
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose
import time
import sys
sys.path.append("/home/undergrads/ut-amrl/object_detection_3d")
sys.path.append("/home/undergrads/ut-amrl/spot_autonomy")
sys.path.append("/home/undergrads/ut-amrl")
import math
import signal
import argparse
import numpy as np
np.float = np.float64  # temp fix for following import https://github.com/eric-wieser/ros_numpy/issues/37
import ros_numpy
from sensor_msgs.msg import PointCloud2
from obj_online import ObjectDetection3D
from scipy.spatial import cKDTree


class WaypointNav:
    WAYPOINTS = {
        "ahg2": [
            (10.4, 74.4, 0.0, 1.0),
            (4.1, 70.8, 0.0, 1.0),
            (12.5, 69.6, 0.0, 1.0),
        ],
        "courtyard": [
            (-10.1, 16.7, 0.0, 1.0),
            (-1.7, 11.2, 0.0, 1.0),
            (11.1, 17.1, 0.0, 1.0),
            (-2.1, 23.1, 0.0, 1.0),
        ],
        "tourguide": [
            (-10.031, 16.859, 0.0, 1.0),  # ahg
            (61.820, -84.904, 0.135, 0.991),  # nhb
            (80.116, -227.031, 0.707, 0.707),  # gdc
            (61.820, -84.904, 0.135, 0.991),  # nhb
        ],
        "emptymap": [
            (0.0, 0.0, -0.3969500385888669, 0.9178402185916115),
            (1.9010463953018188, -2.6240389347076416, 0.6731030619467686, 0.7395486921074802),
            (3.3516833782196045, 0.7416679263114929, -0.9846447535762185, 0.1745700697566109),
        ],
    }

    def __init__(self, stop_time, map):
        self.stop_time = stop_time
        self.map = map
        self.traj = self.WAYPOINTS[map]
        self.next_goal = 0
        self.stop_detected_first_time = -1
        self.INITIAL = True
        self.latest_vlp_points = None
        self.Z_offset_spotbody_to_lidar = 0.32  # meters
        self.MAX_PITCH = 50  # degrees
        self.MAX_YAW = 50  # degrees
        self.obj_detector = ObjectDetection3D(ros_flag=False, open3dvis_bool=False, min_score=0.6)
        self.BODY_POSE_TOPIC = "/spot/body_pose"
        self.PC_TOPIC = "/corrected_velodyne_points"
        rospy.Subscriber('/navigation_goal_status', NavStatusMsg, self.nav_callback, queue_size=1)
        self.pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.speech_pub = rospy.Publisher('/left_black_button_value', Int32, queue_size=1)
        self.body_pose_pub = rospy.Publisher(self.BODY_POSE_TOPIC, Pose, queue_size=1)
        rospy.Subscriber(self.PC_TOPIC, PointCloud2, self.pc_callback, queue_size=1)

    def pc_callback(self, msg):
        pc_cloud = ros_numpy.point_cloud2.pointcloud2_to_array(msg).reshape((1, -1))
        pc_np = np.zeros((pc_cloud.shape[0], pc_cloud.shape[1], 4), dtype=np.float32)
        pc_np[..., 0] = pc_cloud['x']
        pc_np[..., 1] = pc_cloud['y']
        pc_np[..., 2] = pc_cloud['z']
        pc_np[..., 3] = pc_cloud['intensity']
        self.latest_vlp_points = pc_np.reshape((-1, 4))

    @staticmethod
    def get_closest_box(boxes):
        origin = np.array([0, 0, 0]).reshape((1, 3))
        box_centers = boxes[:, :3].reshape((-1, 3))
        tree = cKDTree(box_centers)
        distances, indices = tree.query(origin)
        indices = np.asarray(indices).squeeze()
        return boxes[indices].reshape((1, 7))

    def process_pc(self, pc_np):
        _, pred_boxes, pred_labels, _ = self.obj_detector.detect(pc_np)
        person_mask = np.where(pred_labels == 1, True, False)
        person_boxes = pred_boxes[person_mask].reshape((-1, 7))
        if len(person_boxes) > 0:
            closest_person_box = WaypointNav.get_closest_box(person_boxes)
            corners = self.obj_detector.get_boxes_corners(closest_person_box).squeeze()  # 8 x 3
            top_corners = corners[[3, 4, 5, 6], :]
            mean_top_corner = list(np.mean(top_corners, axis=0).squeeze())
            mean_top_corner[2] -= 0.1  # decrease z to match face height
            mean_top_corner[2] += self.Z_offset_spotbody_to_lidar  # add z offset to transform to spot body frame
            return mean_top_corner
        else:
            return None

    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        return list(quaternion)

    def nav_callback(self, msg):
        pub_msg = PoseStamped()
        if msg.status == 0:  # stopped
            if self.stop_detected_first_time == -1:
                self.stop_detected_first_time = rospy.get_rostime().secs
                if not self.INITIAL:
                    int_msg = Int32()
                    int_msg.data = 1
                    self.speech_pub.publish(int_msg)
                    if self.map == "tourguide":
                        time.sleep(4)
                        x0, y0, z0 = self.process_pc(self.latest_vlp_points)
                        processed_pitch = -math.atan2(z0, math.sqrt(x0**2 + y0**2))
                        processed_yaw = math.atan2(y0, x0)
                        p = Pose()
                        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = WaypointNav.euler_to_quaternion(0.0, processed_pitch, processed_yaw)
                        self.body_pose_pub.publish(p)
            else:
                if (rospy.get_rostime().secs - self.stop_detected_first_time > self.stop_time) or self.INITIAL:
                    if self.INITIAL:
                        time.sleep(10)
                    self.INITIAL = False
                    self.stop_detected_first_time = -1
                    # publish next goal
                    pub_msg.pose.position.x = self.traj[self.next_goal][0]
                    pub_msg.pose.position.y = self.traj[self.next_goal][1]
                    pub_msg.pose.orientation.z = self.traj[self.next_goal][2]
                    pub_msg.pose.orientation.w = self.traj[self.next_goal][3]
                    self.pub.publish(pub_msg)
                    time.sleep(4)
                    self.next_goal += 1
                    self.next_goal %= len(self.traj)
        else:
            self.stop_detected_first_time = -1
            if self.map == "tourguide":
                p = Pose()
                p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = WaypointNav.euler_to_quaternion(0.0, 0.0, 0.0)
                self.body_pose_pub.publish(p)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--stop_time", type=int, required=True, help="Time to stop at each waypoint")
    parser.add_argument("--map", type=str, required=True, help="Map to use: ahg2/courtyard/tourguide/emptymap")
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node('waypoint_nav', anonymous=False)
    obj = WaypointNav(stop_time=args.stop_time, map=args.map)
    time.sleep(1)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS waypoint nav node")
