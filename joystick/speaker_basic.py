#! /usr/bin/env python
import rospy
import tf.transformations
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose
import time
import sys
import math
import os
import signal


class SpeakerBasic:
    def __init__(self):
        self.SPEAKER_BUTTON_TOPIC = "/speaker_button_value"
        self.text_lines = ["Hey how are you?", "I am good", "Have a nice day"]
        self.text_line_counter = 0
        self.prev_val = None
        rospy.Subscriber(self.SPEAKER_BUTTON_TOPIC, Int32, self.speaker_basic_callback, queue_size=1)

    def my_speak(self):
        if self.text_line_counter == len(self.text_lines):
            self.text_line_counter = 0  # start again
        os.system(f'espeak -ven+f3 "{self.text_lines[self.text_line_counter]}"')
        self.text_line_counter += 1

    def speaker_basic_callback(self, msg):
        cur_val = msg.data
        if self.prev_val is None:
            self.prev_val = cur_val
            return
        if self.prev_val == 0 and cur_val == 1:  # button pressed
            self.my_speak()
        self.prev_val = cur_val


if __name__ == '__main__':
    rospy.init_node('speaker_basic')
    speaker_basic = SpeakerBasic()

    def signal_handler(sig, frame):
        print("Ctrl+C detected! Killing speaker_basic node...")
        rospy.sleep(2)
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    time.sleep(1)
    rospy.spin()
