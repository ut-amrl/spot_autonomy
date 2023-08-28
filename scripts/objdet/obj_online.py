import warnings
warnings.filterwarnings("ignore")
import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'  # or any {'0', '1', '2'}
import sys
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), '../..'))
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), '.'))
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../spot_calib'))

import numpy as np
np.float = np.float64  # temp fix for following import https://github.com/eric-wieser/ros_numpy/issues/37
import ros_numpy
from sensor_msgs.msg import PointCloud2
import rospy
from copy import deepcopy
import copy
import json
import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import torch
import open3d as o3d
from easydict import EasyDict
from pcdet.config import cfg, cfg_from_yaml_file
from pcdet.models import build_network, load_data_to_gpu
from pcdet.datasets import DatasetTemplate

import tf.transformations
from std_msgs.msg import String

from spot_calib.spot_calib import SpotCameraCalibration


class DummyDataset(DatasetTemplate):
    def __init__(self, dataset_cfg, class_names, training=False):
        super().__init__(dataset_cfg=dataset_cfg, class_names=class_names, training=training)
        self.dataset_cfg = dataset_cfg


class ObjectDetection3D:
    BBOX_ID_TO_COLOR = [
        (140, 51, 147),  # 0
        (7, 33, 229),  # 1
        (66, 21, 72),  # 2
        (67, 31, 116),  # 3
        (159, 137, 254),  # 4
        (52, 32, 130),  # 5
        (239, 92, 215),  # 6
        (4, 108, 69),  # 7
        (160, 129, 2),  # 8
        (160, 93, 2),  # 9
        (254, 145, 38),  # 10
        (227, 189, 1),  # 11
        (202, 79, 74),  # 12
        (255, 196, 208),  # 13
        (166, 240, 4),  # 14
        (113, 168, 3),  # 15
        (14, 60, 157),  # 16
        (41, 159, 115),  # 17
        (91, 79, 14),  # 18
        (220, 184, 94),  # 19
        (202, 159, 41),  # 20
        (253, 137, 129),  # 21
        (97, 37, 32),  # 22
        (91, 31, 39),  # 23
        (24, 55, 95),  # 24
        (0, 87, 192),  # 25
        (31, 70, 142),  # 26
        (24, 45, 66),  # 27
        (30, 54, 11),  # 28
        (247, 148, 90),  # 29
        (250, 126, 149),  # 30
        (70, 106, 19),  # 31
        (128, 132, 0),  # 32
        (152, 163, 0),  # 33
        (6, 32, 231),  # 34
        (8, 68, 212),  # 35
        (18, 34, 119),  # 36
        (17, 46, 168),  # 37
        (203, 226, 37),  # 38
        (9, 9, 181),  # 39
        (100, 34, 168),  # 40
        (150, 69, 253),  # 41
        (46, 22, 78),  # 42
        (121, 46, 216),  # 43
        (37, 95, 238),  # 44
        (95, 100, 14),  # 45
        (25, 97, 119),  # 46
        (18, 113, 225),  # 47
        (207, 66, 89),  # 48
        (215, 80, 2),  # 49
    ]
    BBOX_ID_TO_COLOR = np.array(BBOX_ID_TO_COLOR).astype(np.float64) / 255.0

    json_dict_template = {
        "3dbbox": [],
        "filetype": "json",
    }
    obj_dict_template = {
        "classId": "",
        "instanceId": "",
        "cX": 0.0,
        "cY": 0.0,
        "cZ": 0.0,
        "h": 0.0,
        "l": 0.0,
        "w": 0.0,
        "labelAttributes": {
            "isOccluded": "None"
        },
        "r": 0.0,
        "p": 0.0,
        "y": 0.0
    }

    def __init__(self, ros_flag=True, open3dvis_bool=True, min_score=0.5):
        self.ros_flag = ros_flag
        self.latest_vlp_points = None
        self.open3dvis_bool = open3dvis_bool
        self.min_score = min_score
        self.cfg_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'coda/pvrcnn_allclass32oracle_coda.yaml')
        self.cfg_dict = EasyDict()
        self.load_params()
        self.dummy_dataset = DummyDataset(dataset_cfg=self.cfg_dict.DATASET_PARAMS,
                                          class_names=self.cfg_dict.CLASS_NAMES,
                                          training=False)
        self.model_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'coda/checkpoint_epoch_50.pth')
        self.model = None
        self.build_model()
        if self.ros_flag:
            rospy.Subscriber("/corrected_velodyne_points", PointCloud2, self.pc_callback, queue_size=1)
            self.demo_pub = rospy.Publisher("/objdet_online/done", String, queue_size=1)
            self.marker_pub = rospy.Publisher('/bbox_viz', MarkerArray, queue_size=1)
            self.fps = 10.0  # max limit is 12 for this model. However incoming pc is at 9.9!
            rospy.Timer(rospy.Duration(1 / self.fps), lambda event: self.detect(self.latest_vlp_points))
        print("Obj Detect setup complete")

    def get_lidar_ext_mat(self):
        """
        Returns: (4 x 4) lidar extrinsic matrix from VLP to Ground
        """
        normal_height = self.cfg_dict.DATASET_PARAMS.LIDAR_HEIGHT
        return SpotCameraCalibration.get_std_trans(cz=-normal_height)

    def projectVLPtoGround(self, vlp_coords):
        """
        Projects set of points in VLP to Ground
        vlp_coords: (N x M) array of points in VLP (where first 3 of M are x, y, z)
        Returns: (N x M) array of points in Ground frame (where first 3 of M are x, y, z)
        """
        ground_coords = deepcopy(vlp_coords)
        vlp_coords_3d = np.array(vlp_coords[:, :3]).astype(np.float64).reshape((-1, 3))
        vlp_coords_4d = SpotCameraCalibration.get_homo_from_ordinary(vlp_coords_3d)
        M_ext = self.get_lidar_ext_mat()
        ground_coords_4d = (M_ext @ vlp_coords_4d.T).T
        ground_coords_3d = SpotCameraCalibration.get_ordinary_from_homo(ground_coords_4d)
        ground_coords[:, :3] = ground_coords_3d
        return ground_coords

    def projectGroundtoVLP(self, ground_coords):
        """
        Projects set of points in Ground to VLP
        ground_coords: (N x M) array of points in Ground frame (where first 3 of M are x, y, z)
        Returns: (N x M) array of points in VLP frame (where first 3 of M are x, y, z)
        """
        vlp_coords = deepcopy(ground_coords)
        ground_coords_3d = np.array(ground_coords[:, :3]).astype(np.float64).reshape((-1, 3))
        ground_coords_4d = SpotCameraCalibration.get_homo_from_ordinary(ground_coords_3d)
        M_ext = np.linalg.inv(self.get_lidar_ext_mat())
        vlp_coords_4d = (M_ext @ ground_coords_4d.T).T
        vlp_coords_3d = SpotCameraCalibration.get_ordinary_from_homo(vlp_coords_4d)
        vlp_coords[:, :3] = vlp_coords_3d
        return vlp_coords

    def process_data(self, pc_np):
        points = deepcopy(pc_np)
        # points[:, 2] = points[:, 2] + np.array(1.6 - self.cfg_dict.DATASET_PARAMS.LIDAR_HEIGHT, dtype=np.float32)
        points = self.projectVLPtoGround(points)
        input_dict = {
            'points': points,
            'frame_id': 0,
        }
        data_dict = self.dummy_dataset.prepare_data(data_dict=input_dict)
        return self.dummy_dataset.collate_batch([data_dict])

    def load_params(self):
        cfg_from_yaml_file(self.cfg_path, cfg, os.path.dirname(os.path.realpath(self.cfg_path)))
        self.cfg_dict.DATASET_PARAMS = cfg.DATA_CONFIG
        self.cfg_dict.CLASS_NAMES = cfg.CLASS_NAMES
        self.cfg_dict.MODEL_PARAMS = cfg.MODEL

    def build_model(self):
        self.model = build_network(model_cfg=self.cfg_dict.MODEL_PARAMS,
                                   num_class=len(self.cfg_dict.CLASS_NAMES),
                                   dataset=self.dummy_dataset)
        self.model.load_params_from_file(filename=self.model_path,
                                         logger=None,
                                         to_cpu=True)
        self.model.cuda()
        self.model.eval()

    @staticmethod
    def viz_open3d(points, ref_boxes=None, ref_labels=None, ref_scores=None, point_colors=None, draw_origin=True):
        vis = o3d.visualization.Visualizer()
        vis.create_window()
        vis.get_render_option().point_size = 1.0
        vis.get_render_option().background_color = np.zeros(3).astype(np.float64) / 255.0
        if draw_origin:
            axis_pcd = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0, 0, 0])
            vis.add_geometry(axis_pcd)
        pts = o3d.geometry.PointCloud()
        pts.points = o3d.utility.Vector3dVector(points[:, :3])
        vis.add_geometry(pts)
        if point_colors is None:
            pts.colors = o3d.utility.Vector3dVector(np.ones((points.shape[0], 3)).astype(np.float64))
        else:
            pts.colors = o3d.utility.Vector3dVector(point_colors)

        if ref_boxes is not None:
            ObjectDetection3D.draw_box(vis=vis, boxes=ref_boxes, color=(0, 1, 0), ref_labels=ref_labels)
        vis.run()
        vis.destroy_window()

    @staticmethod
    def draw_box(vis, boxes, color=(0, 1, 0), ref_labels=None):
        for i in range(boxes.shape[0]):
            line_set, box3d = ObjectDetection3D.translate_boxes_to_open3d_instance(boxes[i])
            if ref_labels is None:
                line_set.paint_uniform_color(color)
            else:
                line_set.paint_uniform_color(ObjectDetection3D.BBOX_ID_TO_COLOR[ref_labels[i]])
            vis.add_geometry(line_set)

    @staticmethod
    def translate_boxes_to_open3d_instance(box, line_set=None):
        """
                4-------- 6
            /|         /|
            5 -------- 3 .
            | |        | |
            . 7 -------- 1
            |/         |/
            2 -------- 0
        """
        center = box[0:3]
        lwh = box[3:6]
        axis_angles = np.array([0, 0, box[6] + 1e-10])
        rot = o3d.geometry.get_rotation_matrix_from_axis_angle(axis_angles)
        box3d = o3d.geometry.OrientedBoundingBox(center, rot, lwh)
        if line_set is None:
            line_set = o3d.geometry.LineSet.create_from_oriented_bounding_box(box3d)
        lines = np.asarray(line_set.lines)
        lines = np.concatenate([lines, np.array([[1, 4], [7, 6]])], axis=0)
        line_set.lines = o3d.utility.Vector2iVector(lines)
        return line_set, box3d

    @staticmethod
    def get_boxes_corners(boxes):
        """
        boxes: (N x 7) array of boxes
        Returns: (N x 8 x 3) array of corners of boxes
        """
        corners = np.zeros((boxes.shape[0], 8, 3), dtype=np.float32)
        for box_idx, box in enumerate(boxes):
            center = box[:3]
            lwh = box[3:6]
            axis_angles = np.array([0, 0, box[6] + 1e-10])
            rot = o3d.geometry.get_rotation_matrix_from_axis_angle(axis_angles)
            box3d = o3d.geometry.OrientedBoundingBox(center, rot, lwh)
            crns = np.asarray(box3d.get_box_points())
            corners[box_idx] = crns
        return corners

    def transform_back(self, in_pred_dicts, in_data_dict):
        """
        Transforms back the predicted boxes and pointcloud from Kitti VLP to Normal VLP
        """
        pred_dicts = deepcopy(in_pred_dicts)
        data_dict = deepcopy(in_data_dict)
        pts = data_dict['points'][:, 1:].cpu().numpy()
        # pts[:, 2] = pts[:, 2] - np.array(1.6 - self.cfg_dict.DATASET_PARAMS.LIDAR_HEIGHT, dtype=np.float32)
        pts = self.projectGroundtoVLP(pts)
        data_dict['points'][:, 1:] = torch.from_numpy(pts).cuda()

        pred_boxes = pred_dicts[0]['pred_boxes'].cpu().numpy()
        # pred_boxes[:, 2] = pred_boxes[:, 2] - np.array(1.6 - self.cfg_dict.DATASET_PARAMS.LIDAR_HEIGHT, dtype=np.float32)
        pred_boxes = self.projectGroundtoVLP(pred_boxes)
        pred_dicts[0]['pred_boxes'] = torch.from_numpy(pred_boxes).cuda()
        return pred_dicts, data_dict

    @staticmethod
    def get_marker(box_idx, obj_dict, label_idx):
        marker = Marker()
        marker.header.frame_id = "velodyne"
        marker.id = box_idx
        marker.type = marker.CUBE
        marker.action = marker.ADD
        # set position of the cube
        marker.pose.position.x = obj_dict["cX"]
        marker.pose.position.y = obj_dict["cY"]
        marker.pose.position.z = obj_dict["cZ"]
        # set orientation of the cube using yaw
        quat = tf.transformations.quaternion_from_euler(0, 0, obj_dict["y"])
        marker.pose.orientation.x = quat[0]
        marker.pose.orientation.y = quat[1]
        marker.pose.orientation.z = quat[2]
        marker.pose.orientation.w = quat[3]
        # set dimensions (assuming they are provided)
        marker.scale.x = obj_dict["l"]
        marker.scale.y = obj_dict["w"]
        marker.scale.z = obj_dict["h"]
        # set color (be sure to set alpha to something non-zero if you want to see it)
        color = ObjectDetection3D.BBOX_ID_TO_COLOR[label_idx]
        marker.color.a = 0.6
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        return marker

    @staticmethod
    def filter_preds(pred_boxes, pred_labels, pred_scores, min_score=0.5):
        """
        Filters out predictions with score less than min_score
        """
        mask = (pred_scores > min_score).squeeze()
        pred_boxes = pred_boxes[mask]
        pred_labels = pred_labels[mask]
        pred_scores = pred_scores[mask]
        return pred_boxes, pred_labels, pred_scores

    def detect(self, pc_np, event=None):
        if pc_np is None:
            return
        cur_pc_np = deepcopy(pc_np)
        with torch.no_grad():
            data_dict = self.process_data(cur_pc_np)
            load_data_to_gpu(data_dict)
            pred_dicts, _ = self.model.forward(data_dict)
            pred_dicts, data_dict = self.transform_back(pred_dicts, data_dict)

            # box: x y z l w h yaw
            data_points = data_dict['points'][:, 1:].cpu().numpy().reshape((-1, 3))
            pred_boxes = pred_dicts[0]['pred_boxes'].cpu().numpy().reshape((-1, 7))
            pred_labels = pred_dicts[0]['pred_labels'].cpu().numpy().reshape((-1, 1)).squeeze()
            pred_labels = pred_labels - 1  # 1 to 50 -> 0 to 49
            pred_scores = pred_dicts[0]['pred_scores'].cpu().numpy().reshape((-1, 1)).squeeze()
            pred_boxes, pred_labels, pred_scores = ObjectDetection3D.filter_preds(pred_boxes, pred_labels, pred_scores, min_score=self.min_score)

            if self.ros_flag:
                json_dict = copy.deepcopy(ObjectDetection3D.json_dict_template)
                delete_marker = Marker()
                delete_marker.action = Marker.DELETEALL
                delete_all_markers = MarkerArray()
                delete_all_markers.markers.append(delete_marker)
                self.marker_pub.publish(delete_all_markers)
                markerArray = MarkerArray()
                markerArray.markers = []
                for box_idx, box in enumerate(pred_boxes):  # loop over all predicted boxes
                    label_idx = pred_labels[box_idx]
                    label_name = self.cfg_dict.CLASS_NAMES[label_idx]
                    obj_dict = copy.deepcopy(ObjectDetection3D.obj_dict_template)
                    obj_dict["cX"] = float(box[0])
                    obj_dict["cY"] = float(box[1])
                    obj_dict["cZ"] = float(box[2])
                    obj_dict["l"] = float(box[3])
                    obj_dict["w"] = float(box[4])
                    obj_dict["h"] = float(box[5])
                    obj_dict["y"] = float(box[6])
                    obj_dict["classId"] = label_name
                    obj_dict["instanceId"] = "%s:%i" % (label_name, box_idx)
                    json_dict["3dbbox"].append(obj_dict)
                    markerArray.markers.append(ObjectDetection3D.get_marker(box_idx, obj_dict, label_idx))
                t_msg = String()
                t_msg.data = json.dumps(json_dict)
                self.demo_pub.publish(t_msg)
                self.marker_pub.publish(markerArray)

            if self.open3dvis_bool:
                ObjectDetection3D.viz_open3d(points=data_points,
                                             ref_boxes=pred_boxes,
                                             ref_labels=pred_labels,
                                             ref_scores=pred_scores)
        return data_points, pred_boxes, pred_labels, pred_scores

    def pc_callback(self, msg):
        pc_cloud = ros_numpy.point_cloud2.pointcloud2_to_array(msg).reshape((1, -1))
        pc_np = np.zeros((pc_cloud.shape[0], pc_cloud.shape[1], 4), dtype=np.float32)
        pc_np[..., 0] = pc_cloud['x']
        pc_np[..., 1] = pc_cloud['y']
        pc_np[..., 2] = pc_cloud['z']
        pc_np[..., 3] = pc_cloud['intensity']
        self.latest_vlp_points = pc_np.reshape((-1, 4))


if __name__ == "__main__":
    ROS_FLAG = True
    if ROS_FLAG:
        rospy.init_node('obj_online')
    e = ObjectDetection3D(ros_flag=ROS_FLAG, open3dvis_bool=False, min_score=0.6)
    if ROS_FLAG:
        rospy.spin()
    # bin_list = os.listdir("/home/dynamo/AMRL_Research/Projects/neurosymbolic_contingency/scripts/ns/data/bins")
    # bin_list.sort()
    # bin_fullpaths = [os.path.join("/home/dynamo/AMRL_Research/Projects/neurosymbolic_contingency/scripts/ns/data/bins", b) for b in bin_list]
    # # idx = 1
    # for idx in range(len(bin_fullpaths)):
    #     points = np.fromfile(bin_fullpaths[idx], dtype=np.float32).reshape((-1, 4))
    #     _ = e.detect(points)
