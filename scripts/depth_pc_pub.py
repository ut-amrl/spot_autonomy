#!/usr/bin/env python3

import cv2
import numpy as np
np.float = np.float64  # temp fix for following import https://github.com/eric-wieser/ros_numpy/issues/37
from sensor_msgs.msg import PointCloud2, CompressedImage, PointField, Image
import rospy
from cv_bridge import CvBridge
import torch
import time
import yaml
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
import math
from repos.depthany2.metric_depth.depth_anything_v2.dpt import DepthAnythingV2
import argparse
torch.backends.cuda.matmul.allow_tf32 = True


class ImageDepthLidar:
    def __init__(self, cam_intrinsics_filepath: str, cam_extrinsics_filepath: str, lidar_actual_extrinsics_filepath: str,
                 depth_image_topic: str, rgb_image_topic: str, point_cloud_topic: str,
                 mode: str, device: str = None):
        if device is not None:
            self.DEVICE = torch.device(device)
        else:
            self.DEVICE = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.MODE = mode
        self.cam_intrinsics_filepath = cam_intrinsics_filepath
        self.cam_extrinsics_filepath = cam_extrinsics_filepath
        self.lidar_actual_extrinsics_filepath = lidar_actual_extrinsics_filepath
        self.load_params()
        self.setup_()

        self.cv_bridge = CvBridge()
        self.latest_depth_img_cv2_np = None
        self.latest_rgb_img_cv2_np = None
        rospy.Subscriber(depth_image_topic, CompressedImage, self.depth_callback, queue_size=1)
        rospy.Subscriber(rgb_image_topic, Image, self.rgb_callback, queue_size=1)
        self.pc_pub = rospy.Publisher(point_cloud_topic, PointCloud2, queue_size=1)
        rospy.Timer(rospy.Duration(1 / 10), lambda event: self.main(self.latest_depth_img_cv2_np, self.latest_rgb_img_cv2_np))

    def setup_(self):
        model_configs = {
            'vits': {'encoder': 'vits', 'features': 64, 'out_channels': [48, 96, 192, 384]},
            'vitb': {'encoder': 'vitb', 'features': 128, 'out_channels': [96, 192, 384, 768]},
            'vitl': {'encoder': 'vitl', 'features': 256, 'out_channels': [256, 512, 1024, 1024]}
        }

        encoder = 'vits'  # or 'vits', 'vitb', 'vitl'
        dataset = 'hypersim'  # 'hypersim' for indoor model, 'vkitti' for outdoor model
        max_depth = 20  # 20 for indoor model, 80 for outdoor model
        self.depth_model = DepthAnythingV2(**{**model_configs[encoder], 'max_depth': max_depth})
        self.depth_model.load_state_dict(torch.load(f'repos/depthany2/metric_depth/checkpoints/depth_anything_v2_metric_{dataset}_{encoder}.pth', map_location=self.DEVICE))
        self.depth_model.eval()
        self.depth_model.to(self.DEVICE)

    @torch.inference_mode()
    def main(self, depth_img, rgb_img, event=None):
        with torch.device(self.DEVICE):
            if self.MODE == "cam":
                if depth_img is None:
                    return
                depth_arr = depth_img
            elif self.MODE == "model":
                if rgb_img is None:
                    return
                print("Inferencing depth")
                depth_arr = self.depth_model.infer_image(rgb_img)
                print("Inferencing depth done", depth_arr.shape)
            else:
                raise ValueError("Invalid mode")

            scale_default = 0.6 if self.MODE == "model" else 10
            SCALE = rospy.get_param("/SCALE", scale_default)
            depth_arr = SCALE * depth_arr

            kinect_points = ImageDepthLidar.depth2points(depth_arr, self.cam_intrinsics_dict)
            lidar_points = self.project_points_kinect_to_lidar(kinect_points)

            fields = [
                PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1),
            ]
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = "velodyne"
            ros_pcd = pc2.create_cloud(header, fields, lidar_points)
        self.pc_pub.publish(ros_pcd)

    def rgb_callback(self, msg):
        self.latest_rgb_img_cv2_np = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def depth_callback(self, msg):
        self.latest_depth_img_cv2_np = np.asarray(self.cv_bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough"), dtype=np.float32).squeeze() / 1000
        assert self.latest_depth_img_cv2_np.ndim == 2

    def load_params(self):
        with open(self.cam_intrinsics_filepath, 'r') as f:
            self.cam_intrinsics_dict = yaml.safe_load(f)
        self.cam_intrinsics_dict['camera_matrix'] = np.array(self.cam_intrinsics_dict['camera_matrix']).reshape((3, 3))
        self.cam_intrinsics_dict['dist_coeffs'] = np.array(self.cam_intrinsics_dict['dist_coeffs']).reshape((1, 5)).squeeze()
        self.img_height = self.cam_intrinsics_dict['height']
        self.img_width = self.cam_intrinsics_dict['width']

        with open(self.cam_extrinsics_filepath, 'r') as f:
            self.cam_extrinsics_dict = yaml.safe_load(f)

        with open(self.lidar_actual_extrinsics_filepath, 'r') as f:
            self.lidar_actual_extrinsics_dict = yaml.safe_load(f)

    def M_baselink_to_kinect(self):
        """
        Returns the extrinsic matrix (4 x 4) that transforms from base_link to kinect frame (CCS)
        """
        T1 = ImageDepthLidar.get_std_trans(cx=self.cam_extrinsics_dict['T12']['T1']['X'] / 100,
                                           cy=self.cam_extrinsics_dict['T12']['T1']['Y'] / 100,
                                           cz=self.cam_extrinsics_dict['T12']['T1']['Z'] / 100)
        T2 = ImageDepthLidar.get_std_rot(axis=self.cam_extrinsics_dict['T23']['R1']['axis'],
                                         alpha=np.deg2rad(self.cam_extrinsics_dict['T23']['R1']['alpha']))
        T3 = np.array(self.cam_extrinsics_dict['T23']['R2'])
        T4 = ImageDepthLidar.get_std_rot(axis=self.cam_extrinsics_dict['T23']['R3']['axis'],
                                         alpha=np.deg2rad(self.cam_extrinsics_dict['T23']['R3']['alpha']))
        T5 = ImageDepthLidar.get_std_rot(axis=self.cam_extrinsics_dict['T23']['R4']['axis'],
                                         alpha=np.deg2rad(self.cam_extrinsics_dict['T23']['R4']['alpha']))
        return T5 @ T4 @ T3 @ T2 @ T1

    def M_baselink_to_lidar(self):
        """
        Returns the extrinsic matrix (4 x 4) that transforms from base_link to (true) lidar points frame
        """
        T1 = ImageDepthLidar.get_std_trans(cx=self.lidar_actual_extrinsics_dict['T1']['Trans1']['X'] / 100,
                                           cy=self.lidar_actual_extrinsics_dict['T1']['Trans1']['Y'] / 100,
                                           cz=self.lidar_actual_extrinsics_dict['T1']['Trans1']['Z'] / 100)
        T2 = ImageDepthLidar.get_std_rot(axis=self.lidar_actual_extrinsics_dict['T2']['Rot1']['axis'],
                                         alpha=np.deg2rad(self.lidar_actual_extrinsics_dict['T2']['Rot1']['alpha']))
        T3 = ImageDepthLidar.get_std_rot(axis=self.lidar_actual_extrinsics_dict['T2']['Rot2']['axis'],
                                         alpha=np.deg2rad(self.lidar_actual_extrinsics_dict['T2']['Rot2']['alpha']))
        return (T2 @ T3) @ T1

    def project_points_kinect_to_lidar(self, points_kinect):
        M_kinect_to_lidar = self.M_baselink_to_lidar() @ np.linalg.inv(self.M_baselink_to_kinect())
        return ImageDepthLidar.general_project_A_to_B(points_kinect, M_kinect_to_lidar)

    @staticmethod
    def depth2points(depth_arr_img: np.ndarray, cam_intrinsics_dict):
        FX = cam_intrinsics_dict['camera_matrix'][0, 0]
        FY = cam_intrinsics_dict['camera_matrix'][1, 1]
        CX = cam_intrinsics_dict['camera_matrix'][0, 2]
        CY = cam_intrinsics_dict['camera_matrix'][1, 2]
        K = cam_intrinsics_dict['camera_matrix']
        d = cam_intrinsics_dict['dist_coeffs']
        R = np.eye(3)
        x, y = np.meshgrid(np.arange(depth_arr_img.shape[1]), np.arange(depth_arr_img.shape[0]))
        # undistort pixel coordinates
        pcs_coords = np.stack((x.flatten(), y.flatten()), axis=-1).astype(np.float64)
        undistorted_pcs_coords = cv2.undistortPoints(pcs_coords.reshape(1, -1, 2), K, d, R=R, P=K)
        undistorted_pcs_coords = np.swapaxes(undistorted_pcs_coords, 0, 1).squeeze().reshape((-1, 2))
        x, y = np.split(undistorted_pcs_coords, 2, axis=1)
        x = x.reshape(depth_arr_img.shape[0], depth_arr_img.shape[1])
        y = y.reshape(depth_arr_img.shape[0], depth_arr_img.shape[1])
        # back project (along the camera ray) the pixel coordinates to 3D using the depth
        x = (x - CX) / FX
        y = (y - CY) / FY
        points = np.stack((np.multiply(x, depth_arr_img), np.multiply(y, depth_arr_img), depth_arr_img), axis=-1).reshape(-1, 3)
        return points

    @staticmethod
    def general_project_A_to_B(inp, AtoBmat):
        """
        Project inp from A frame to B
        inp: (N x 3) array of points in A frame
        AtoBmat: (4 x 4) transformation matrix from A to B
        Returns: (N x 3) array of points in B frame
        """
        inp = np.array(inp).astype(np.float64)
        inp_4d = ImageDepthLidar.get_homo_from_ordinary(inp)
        out_4d = (AtoBmat @ inp_4d.T).T
        return ImageDepthLidar.get_ordinary_from_homo(out_4d)

    @staticmethod
    def get_ordinary_from_homo(points_higherD):
        # Scales so that last coord is 1 and then removes last coord
        points_higherD = points_higherD / points_higherD[:, -1].reshape(-1, 1)  # scale by the last coord
        return points_higherD[:, :-1]

    @staticmethod
    def get_homo_from_ordinary(points_lowerD):
        # Append 1 to each point
        ones = np.ones((points_lowerD.shape[0], 1))  # create a column of ones
        return np.hstack([points_lowerD, ones])  # append the ones column to points

    @staticmethod
    def get_std_trans(cx=0, cy=0, cz=0):
        """
        cx, cy, cz are the coords of O_M wrt O_F when expressed in F
        Multiplication goes like M_coords = T * F_coords
        """
        mat = [
            [1, 0, 0, -cx],
            [0, 1, 0, -cy],
            [0, 0, 1, -cz],
            [0, 0, 0, 1]
        ]
        return np.array(mat)

    @staticmethod
    def get_std_rot(axis, alpha):
        """
        axis is either "X", "Y", or "Z" axis of F and alpha is positive acc to right hand thumb rule dirn
        Multiplication goes like M_coords = T * F_coords
        """
        if axis == "X":
            mat = [
                [1, 0, 0, 0],
                [0, math.cos(alpha), math.sin(alpha), 0],
                [0, -math.sin(alpha), math.cos(alpha), 0],
                [0, 0, 0, 1]
            ]
        elif axis == "Y":
            mat = [
                [math.cos(alpha), 0, -math.sin(alpha), 0],
                [0, 1, 0, 0],
                [math.sin(alpha), 0, math.cos(alpha), 0],
                [0, 0, 0, 1]
            ]
        elif axis == "Z":
            mat = [
                [math.cos(alpha), math.sin(alpha), 0, 0],
                [-math.sin(alpha), math.cos(alpha), 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1]
            ]
        else:
            raise ValueError("Invalid axis!")
        return np.array(mat)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', type=str, default='model', help='mode: model or cam')
    parser.add_argument('--device', type=str, default=None, help='device: cuda or cpu')
    args = parser.parse_args(rospy.myargv()[1:])  # Exclude the script name

    rospy.init_node('depth_from_cam', anonymous=False)
    e = ImageDepthLidar(
        cam_intrinsics_filepath="../config/cam_intrinsics_1536.yaml",
        cam_extrinsics_filepath="../config/baselink_to_kinect_extrinsics.yaml",
        lidar_actual_extrinsics_filepath="../config/baselink_to_actual_lidar_extrinsics.yaml",
        depth_image_topic="/camera/depth/image_raw/compressed",
        rgb_image_topic="/camera/rgb/image_raw",
        point_cloud_topic="/camdepth_points",
        mode=args.mode,
        device=args.device
    )
    time.sleep(1)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down faster synapse module")