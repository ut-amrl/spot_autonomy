#!/usr/bin/env python
import os
import sys
sys.path.append("/home/undergrads/ut-amrl/object_detection_3d")
sys.path.append("/home/undergrads/ut-amrl/spot_autonomy")
import numpy as np
import rospy
import time
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from amrl_msgs.msg import NavStatusMsg
import argparse
import subprocess

ANSI_GREEN = "\033[92m"
ANSI_END = "\033[0m"


class DemoWrapper:
    NUM_MAP = {
        'nominalnav': 0,
        'waypoint': 1,
        'bodypose': 2
    }

    def __init__(self, mode, map, stop_time, pose):
        self.mode = mode
        self.mode_num = self.NUM_MAP[self.mode]
        self.map = map
        self.pose = pose
        self.stop_time = stop_time
        self.mode_pub = rospy.Publisher('/demo_mode_num', Int32, queue_size=1)
        rospy.loginfo(ANSI_GREEN + "Demo mode set to: {}".format(self.mode) + ANSI_END)
        rospy.on_shutdown(self.shutdown_hook)
        self.demo_process_node = self.launch_demo_node()

        rospy.Timer(rospy.Duration(1 / 30), self.main)

    def main(self, _):
        msg = Int32()
        msg.data = self.mode_num
        self.mode_pub.publish(msg)

    def shutdown_hook(self):
        if self.demo_process_node is not None:
            self.demo_process_node.terminate()

    def launch_demo_node(self):
        if self.mode == 'nominalnav':
            process = None
        elif self.mode == 'waypoint':
            cmd = ["rosrun", "spot_autonomy", "waypoint.py", "--stop_time", f"{self.stop_time}", "--map", f"{self.map}"]
            process = subprocess.Popen(cmd)
        elif self.mode == 'bodypose':
            cmd = ["rosrun", "spot_autonomy", "bodypose.py", "--mode", f"{self.pose}"]
            process = subprocess.Popen(cmd)
        return process


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", type=str, required=True, help="Demo mode: nominalnav/waypoint/bodypose")
    parser.add_argument("--map", type=str, required=True, help="(waypoint mode) Map to use: ahg2/courtyard/tourguide")
    parser.add_argument("--stop_time", type=int, required=True, help="(waypoint mode) Time to stop at each waypoint")
    parser.add_argument("--pose", type=str, required=True, help="(bodypose mode) Pose mode: auto/manual")
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node('demo_wrapper', anonymous=False)
    obj = DemoWrapper(mode=args.mode, map=args.map, stop_time=args.stop_time, pose=args.pose)
    time.sleep(1)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS demo wrapper node")
