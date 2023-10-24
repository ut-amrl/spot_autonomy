#! /usr/bin/env python
import rospy
from std_msgs.msg import Int32
import time
import sys
import signal
import io
from gtts import gTTS
from pydub import AudioSegment
from pydub.playback import play
import threading
import argparse


class SpeakerBasic:
    def __init__(self, map):
        self.SPEAKER_BUTTON_TOPIC = "/left_black_button_value"
        self.text_file_path = f"waypoint_speech_{map}.txt"
        self.text_lines = []
        self.lock = threading.Lock()  # so that the callback is not called multiple times if the first one is running

        with open(self.text_file_path, 'r') as file:
            # Read all lines and store them in a list
            self.text_lines = file.readlines()

        self.text_lines = [line.strip() for line in self.text_lines]
        self.text_line_counter = 0
        self.prev_val = None
        rospy.Subscriber(self.SPEAKER_BUTTON_TOPIC, Int32, self.wrapper_speaker_basic_callback, queue_size=1)

    def wrapper_speaker_basic_callback(self, msg):
        if self.lock.acquire(False):
            try:
                self.speaker_basic_callback(msg)
            finally:
                self.lock.release()

    def my_speak(self):
        if self.text_line_counter >= len(self.text_lines):
            self.text_line_counter = 0  # start again
        tts = gTTS(self.text_lines[self.text_line_counter])  # Use gTTS to convert text to speech

        # Save the speech to an in-memory bytes buffer
        fp = io.BytesIO()
        tts.write_to_fp(fp)
        fp.seek(0)

        audio = AudioSegment.from_file(fp, format='mp3')  # Create an AudioSegment object from the bytes buffer
        play(audio)
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
    parser = argparse.ArgumentParser()
    parser.add_argument("--map", type=str, required=True, help="Map to use: ahg2/courtyard/tourguide/emptymap1/emptymap2")
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node('speaker_basic')
    speaker_basic = SpeakerBasic(args.map)

    def signal_handler(sig, frame):
        print("Ctrl+C detected! Killing speaker_basic node...")
        rospy.sleep(2)
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    time.sleep(1)
    rospy.spin()
