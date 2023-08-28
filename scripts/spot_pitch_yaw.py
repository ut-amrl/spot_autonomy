#! /usr/bin/env python
import rospy
import tf.transformations
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose
import time
import sys
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), '../spot_calib'))
import math
import signal
import argparse
import numpy as np
np.float = np.float64  # temp fix for following import https://github.com/eric-wieser/ros_numpy/issues/37
import ros_numpy
from sensor_msgs.msg import PointCloud2
from objdet.obj_online import ObjectDetection3D
from scipy.spatial import cKDTree


class SpotPitchYaw:
    def __init__(self, mode="auto"):
        self.mode = mode
        self.BODY_POSE_TOPIC = "/spot/body_pose"
        self.PITCH_YAW_TOPIC = "/pitch_yaw_axes_values"
        self.PC_TOPIC = "/corrected_velodyne_points"
        self.anchor_head_point = None
        self.Z_offset_spotbody_to_lidar = 0.32  # meters
        self.MAX_PITCH = 50  # degrees
        self.MAX_YAW = 50  # degrees
        self.obj_detector = ObjectDetection3D(ros_flag=False, open3dvis_bool=False, min_score=0.6)
        self.current_pitch_yaw = None
        self.latest_vlp_points = None  # x y z
        self.body_pose_pub = rospy.Publisher(self.BODY_POSE_TOPIC, Pose, queue_size=1)
        rospy.Subscriber(self.PITCH_YAW_TOPIC, Float32MultiArray, self.pitch_yaw_callback, queue_size=1)
        rospy.Subscriber(self.PC_TOPIC, PointCloud2, self.pc_callback, queue_size=1)

    @staticmethod
    def get_closest_box(boxes):
        origin = np.array([0, 0, 0]).reshape((1, 3))
        box_centers = boxes[:, :3].reshape((-1, 3))
        tree = cKDTree(box_centers)
        distances, indices = tree.query(origin)
        indices = np.asarray(indices).squeeze()
        return boxes[indices].reshape((1, 7))

    def pc_callback(self, msg):
        pc_cloud = ros_numpy.point_cloud2.pointcloud2_to_array(msg).reshape((1, -1))
        pc_np = np.zeros((pc_cloud.shape[0], pc_cloud.shape[1], 4), dtype=np.float32)
        pc_np[..., 0] = pc_cloud['x']
        pc_np[..., 1] = pc_cloud['y']
        pc_np[..., 2] = pc_cloud['z']
        pc_np[..., 3] = pc_cloud['intensity']
        self.latest_vlp_points = pc_np.reshape((-1, 4))

    def process_pc(self, pc_np):
        _, pred_boxes, pred_labels, _ = self.obj_detector.detect(pc_np)
        person_mask = np.where(pred_labels == 1, True, False)
        person_boxes = pred_boxes[person_mask].reshape((-1, 7))
        if len(person_boxes) > 0:
            closest_person_box = SpotPitchYaw.get_closest_box(person_boxes)
            corners = self.obj_detector.get_boxes_corners(closest_person_box).squeeze()  # 8 x 3
            top_corners = corners[[3, 4, 5, 6], :]
            mean_top_corner = list(np.mean(top_corners, axis=0).squeeze())
            mean_top_corner[2] -= 0.1  # decrease z to match face height
            mean_top_corner[2] += self.Z_offset_spotbody_to_lidar  # add z offset to transform to spot body frame
            return mean_top_corner
        else:
            return None

    @staticmethod
    def convert_to_rad(degrees):
        return degrees * (math.pi / 180)

    @staticmethod
    def map_to_range(x, max_in, max_out, clip=True):
        in_min = -max_in
        in_max = max_in
        out_min = -max_out
        out_max = max_out

        mapped_value = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
        if clip:
            mapped_value = max(min(mapped_value, out_max), out_min)
        return mapped_value

    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        return list(quaternion)

    def pitch_yaw_callback(self, msg):
        self.current_pitch_yaw = list(msg.data)
        raw_pitch = self.current_pitch_yaw[0]
        raw_yaw = self.current_pitch_yaw[1]
        p = Pose()

        if self.mode == "manual":
            # Processing yaw (incoming raw_yaw is in range ~[-0.7, 0.7])
            processed_yaw = SpotPitchYaw.convert_to_rad(SpotPitchYaw.map_to_range(raw_yaw, 0.7, self.MAX_YAW, clip=False))
            # Processing pitch (incoming raw_pitch is in range ~[-0.7, 0.7]). But need to take into account right_toggle's state first
            if raw_pitch < -0.7:  # right toggle is up (approx condition)
                processed_pitch = 0.0  # no pitch
            else:
                processed_pitch = SpotPitchYaw.convert_to_rad(SpotPitchYaw.map_to_range(raw_pitch, 0.7, self.MAX_PITCH, clip=False))
            processed_pitch = -processed_pitch  # to ensure that raw_pitch > 0 corresponds to robot facing up
        elif self.mode == "auto":
            # only use the yaw toggle: away is auto (towards nearest human head), and else no pitch and yaw
            if self.latest_vlp_points is None:
                return
            if raw_yaw > 0 and self.anchor_head_point is None:
                # processed_pitch is a radian angle such that it is positive when the robot is facing down
                # processed_yaw is a radian angle such that it is positive when the robot is facing left
                self.anchor_head_point = self.process_pc(self.latest_vlp_points)
            
            if raw_yaw > 0 and self.anchor_head_point is not None:
                x0, y0, z0 = self.anchor_head_point
                processed_pitch = -math.atan2(z0, math.sqrt(x0**2 + y0**2))
                processed_yaw = math.atan2(y0, x0)
            else:
                processed_pitch = 0.0
                processed_yaw = 0.0
                self.anchor_head_point = None
        else:
            raise ValueError("Invalid mode. Options: auto, manual")
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = SpotPitchYaw.euler_to_quaternion(0.0, processed_pitch, processed_yaw)
        self.body_pose_pub.publish(p)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", default="auto", type=str, help="Mode for pitch and yaw control. Options: auto, manual")
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node('spot_pitch_yaw')
    spot_pitch_yaw = SpotPitchYaw(mode=args.mode)

    def signal_handler(sig, frame):
        print("Ctrl+C detected! Killing spot_pitch_yaw node...")
        rospy.sleep(2)
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    time.sleep(1)
    rospy.spin()
