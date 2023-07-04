#! /usr/bin/env python
import rospy
import time
import subprocess
import argparse
from std_msgs.msg import String


class MotionReplay:
    def __init__(self, bag_file):
        self.TOPICS = ['/cmd_vel', '/spot/body_pose', '/speaker_button_value']
        self.bag_file = bag_file
        self.process = None
        self.MOTION_MODE_TOPIC = "/motion_mode"
        self.motion_mode_pub = rospy.Publisher(self.MOTION_MODE_TOPIC, String, queue_size=1)
        self.init_motion_mode()

    def init_motion_mode(self):
        while self.motion_mode_pub.get_num_connections() < 1:
            time.sleep(0.1)
        msg = String()
        msg.data = "bag"
        self.motion_mode_pub.publish(msg)
        time.sleep(2)

    def play_bag(self):
        self.process = subprocess.Popen(['rosbag', 'play', self.bag_file, '--topics'] + self.TOPICS)
        self.process.communicate()

    def stop_bag(self):
        if self.process:
            self.process.terminate()
            time.sleep(2)
            # Check if the process has terminated, if not, kill it
            if self.process.poll() is None:
                self.process.kill()
                time.sleep(2)
        msg = String()
        msg.data = "joy"
        self.motion_mode_pub.publish(msg)
        time.sleep(2)


if __name__ == '__main__':
    rospy.init_node('motion_replay')

    parser = argparse.ArgumentParser()
    parser.add_argument('--bag', type=str, help='Path to the ROS bag file to replay.', required=True)
    args = parser.parse_args(rospy.myargv()[1:])

    motion_replay = MotionReplay(args.bag)

    try:
        motion_replay.play_bag()
    except KeyboardInterrupt:
        # Gracefully handle CTRL+C by terminating the subprocess
        print("\nCTRL+C pressed. Terminating the motion_replay node.")
        motion_replay.stop_bag()
    finally:
        print("Exiting motion_replay script.")
        time.sleep(1)
