#!/usr/bin/env python3
import argparse
import rospy
import threading
import time
import torch

from cv_bridge import CvBridge

import std_msgs
from sensor_msgs.msg import Image
from std_msgs.msg import String

from gsam_depth import FastGSAM
from ultralytics import YOLO

import torchvision.transforms as T

class ImageGSAM:
    def __init__(self, rgb_image_topic: str, output_topic: str, device: str = None, model='gsam', use_threading=True):
        if device is not None:
            self.DEVICE = torch.device(device)
        else:
            self.DEVICE = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        
        if self.DEVICE == 'cuda':
            assert torch.cuda.is_available(), f"You asked for device=cuda, but it aint available"
        
        self.latest_rgb_img_cv2_np = torch.rand(1, 3, 2560, 1440)
        self.use_threading = use_threading
        
        self.data_lock = threading.Lock()
        self.model_lock = threading.Lock()

        self.invocation_count = 0

        self.model = model
        assert self.model == 'gsam' or self.model.startswith('yolo')
        
        self.setup_model()
        rospy.set_param('~gsam_classes', 'door')

        self.cv_bridge = CvBridge()
        rospy.Subscriber(rgb_image_topic, Image, self.rgb_callback, queue_size=1)
        self.ann_pub = rospy.Publisher(output_topic, std_msgs.msg.String, queue_size=1)
        self.threading_stats = rospy.Publisher('/gsam_stats', std_msgs.msg.String, queue_size=1)
        self.all_threads = []
        rospy.Timer(rospy.Duration(1), lambda event: self.main())

    def setup_model(self):
        if self.model == 'gsam':
            self.gsam_model = FastGSAM(device=self.DEVICE)
        elif self.model.startswith('yolo'):
            model = YOLO(self.model)
            self.yolo_model = model.to(self.DEVICE)

    @torch.inference_mode()
    def main(self, event=None):
        # cleanup
        if self.use_threading:
            self.all_threads = list(filter(lambda thread: thread.is_alive(), self.all_threads))
            self.threading_stats.publish(f"Total {len(self.all_threads)} active\n")

            if len(self.all_threads) >= 9:
                self.threading_stats.publish(f"Dropping event.")
                return
        
        with self.data_lock:
            rgb_img = self.latest_rgb_img_cv2_np
            # self.latest_rgb_img_cv2_np = None
        
        if rgb_img is None:
            print("Dropping image since it is None")
            return
        
        classes = rospy.get_param('~gsam_classes', 'door,chair')
        classes = classes.split(',')
        
        args=(rgb_img, classes, self.invocation_count)
        self.invocation_count += 1
        
        if self.model == 'gsam':
            function_to_call = self.gsam_inference
        elif self.model.startswith('yolo'):
            function_to_call = self.yolo_inference
        
        if self.use_threading:
            thread = threading.Thread(target=function_to_call, args=args)
            thread.start()
            self.all_threads.append(thread)
        else:
            function_to_call(*args)

    @torch.inference_mode()
    def yolo_inference(self, rgb_img, classes, invocation_id):
        rgb_img = torch.tensor(rgb_img)
        rgb_img = rgb_img.permute((2, 0, 1))
        rgb_img = T.RandomAffine(degrees=45, translate=(0.1, 0.1), scale=(0.9, 1.1), shear=(-10, 10))(rgb_img)
        rgb_img = rgb_img.reshape(1, *rgb_img.shape)
        rgb_img = rgb_img.float() / 255

        with self.model_lock:
            stats = self.yolo_model(rgb_img, verbose=False)[0].speed
        
        self.ann_pub.publish(str(stats))

    @torch.inference_mode()
    def gsam_inference(self, rgb_img, classes, invocation_id):
        start = time.time()
        with self.model_lock:
            for clsname in classes:
                ann_img, detections, per_class_mask = self.gsam_model.predict_and_segment_on_image(img=rgb_img, text_prompts=clsname)
        end = time.time()
        self.ann_pub.publish(f"start={start} end={end}\n")
        
    def rgb_callback(self, msg):
        with self.data_lock:
            self.latest_rgb_img_cv2_np = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")[:,:,:3]

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--device', type=str, default='cuda', help='device: cuda or cpu')
    parser.add_argument('--model', type=str, default='yolo11n-obb.pt', help='What model to use? available: [gsam, yolo11n.pt , yolo11n-pose.pt, yolo11n-seg.pt (200ms), yolo11n-obb.pt (200ms), yolo11n-cls.pt]')
    
    args = parser.parse_args(rospy.myargv()[1:])  # Exclude the script name

    rospy.init_node('depth_from_cam', anonymous=False)
    e = ImageGSAM(
        rgb_image_topic="/camera/rgb/image_raw",
        output_topic="/gsam_output",
        device=args.device,
        model=args.model
    )
    time.sleep(1)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down faster synapse module")
