#! /usr/bin/env python
import rospy
import time
import subprocess
import argparse


class MotionReplay:
    def __init__(self, bag_file):
        self.TOPICS = ['/cmd_vel', '/spot/body_pose', '/speaker_button_value']
        self.bag_file = bag_file

    def play_bag(self):
        process = subprocess.Popen(['rosbag', 'play', self.bag_file] + self.TOPICS)
        process.communicate()


if __name__ == '__main__':
    rospy.init_node('motion_replay')

    parser = argparse.ArgumentParser()
    parser.add_argument('--bag', type=str, help='Path to the ROS bag file to replay.', required=True)
    args = parser.parse_args(rospy.myargv()[1:])

    motion_replay = MotionReplay(args.bag)
    motion_replay.play_bag()

    time.sleep(1)
