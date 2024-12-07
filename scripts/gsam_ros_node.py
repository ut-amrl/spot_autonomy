#!/usr/bin/env python3
import argparse
import rospy
import time
import torch

from cv_bridge import CvBridge

import std_msgs
from sensor_msgs.msg import Image

from gsam_depth import FastGSAM

class ImageGSAM:
    def __init__(self, rgb_image_topic: str, output_topic: str, device: str = None, classes = ['door']):
        if device is not None:
            self.DEVICE = torch.device(device)
        else:
            self.DEVICE = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        if self.DEVICE == 'cuda':
            assert torch.cuda.is_available(), f"You asked for device=cuda, but it aint available"
        self.setup_model()
        self.classes = classes

        self.cv_bridge = CvBridge()
        self.latest_rgb_img_cv2_np = None
        rospy.Subscriber(rgb_image_topic, Image, self.rgb_callback, queue_size=1)
        self.ann_pub = rospy.Publisher(output_topic, std_msgs.msg.String, queue_size=1)
        rospy.Timer(rospy.Duration(1 / 10), lambda event: self.main(self.latest_rgb_img_cv2_np))

    def setup_model(self):
        self.gsam_model = FastGSAM(device=self.DEVICE)

    @torch.inference_mode()
    def main(self, rgb_img, event=None):
        start_time = time.time()
        ann_img, detections, per_class_mask = self.gsam_model.predict_and_segment_on_image(img=rgb_img, text_prompts=self.classes)
        end_time = time.tim()
        print("GSAM time:", end_time - start_time)
	# self.ann_pub.publish(ros_pcd)

    def rgb_callback(self, msg):
        self.latest_rgb_img_cv2_np = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--device', type=str, default=None, help='device: cuda or cpu')
    args = parser.parse_args(rospy.myargv()[1:])  # Exclude the script name

    rospy.init_node('depth_from_cam', anonymous=False)
    e = ImageGSAM(
        rgb_image_topic="/camera/rgb/image_raw",
        output_topic="/gsam_output",
        device=args.device,
        classes=['door']
    )
    time.sleep(1)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down faster synapse module")
