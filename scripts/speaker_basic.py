#! /usr/bin/env python
import rospy
import time
from std_msgs.msg import Int32
import os
import sys
import signal
import shutil
from gtts import gTTS
from pydub import AudioSegment
from pydub.playback import play


class SpeakerBasic:
    def __init__(self):
        self.SPEAKER_BUTTON_TOPIC = "/speaker_button_value"
        self.text_file_path = "speech.txt"
        self.text_lines = []
        self.audio_dir = "audio/"

        with open(self.text_file_path, 'r') as file:
            # Read all lines and store them in a list
            self.text_lines = file.readlines()

        self.text_lines = [line.strip() for line in self.text_lines]
        self.text_line_counter = 0
        self.prev_val = None

        # Create audio directory if it does not exist
        if not os.path.exists(self.audio_dir):
            os.makedirs(self.audio_dir)

        # Generate mp3 files
        self.generate_mp3s()

        rospy.Subscriber(self.SPEAKER_BUTTON_TOPIC, Int32, self.speaker_basic_callback, queue_size=1)

    def generate_mp3s(self):
        for idx, line in enumerate(self.text_lines):
            tts = gTTS(line)
            tts.save(os.path.join(self.audio_dir, f"{idx + 1}.mp3"))
        print("------------------------------------------Generated all mp3s")

    def my_speak(self):
        if self.text_line_counter == len(self.text_lines):
            self.text_line_counter = 0  # start again

        audio_path = os.path.join(self.audio_dir, f"{self.text_line_counter + 1}.mp3")
        audio = AudioSegment.from_file(audio_path)
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
    rospy.init_node('speaker_basic')
    speaker_basic = SpeakerBasic()

    def signal_handler(sig, frame):
        print("Ctrl+C detected! Deleting audio directory and killing speaker_basic node...")
        shutil.rmtree(speaker_basic.audio_dir)
        rospy.sleep(2)
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    time.sleep(1)
    rospy.spin()
