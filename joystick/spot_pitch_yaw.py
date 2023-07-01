#! /usr/bin/env python
import rospy
import tf.transformations
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose
import time
import sys
import math
import signal


class SpotPitchYaw:
    def __init__(self):
        self.BODY_POSE_TOPIC = "/spot/body_pose"
        self.PITCH_YAW_TOPIC = "/pitch_yaw_axes_values"
        self.MAX_PITCH = 60 # degrees
        self.MAX_YAW = 60 # degrees
        self.current_pitch_yaw = None
        self.body_pose_pub = rospy.Publisher(self.BODY_POSE_TOPIC, Pose, queue_size=1)
        rospy.Subscriber(self.PITCH_YAW_TOPIC, Float32MultiArray, self.pitch_yaw_callback, queue_size=1)

    def convert_to_rad(self, degrees):
        return degrees * (math.pi / 180)

    def map_to_range(self, x, max_in, max_out):
        in_min = -max_in
        in_max = max_in
        out_min = -max_out
        out_max = max_out
        
        mapped_value = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
        return mapped_value
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        return list(quaternion)

    def pitch_yaw_callback(self, msg):
        self.current_pitch_yaw = list(msg.data)
        raw_pitch = self.current_pitch_yaw[0]
        raw_yaw = self.current_pitch_yaw[1]
        
        # Processing yaw (incoming raw_yaw is in range ~[-0.7, 0.7])
        processed_yaw = self.map_to_range(raw_yaw, 0.75, self.convert_to_rad(self.MAX_YAW))  # 0.75 to be safer side

        # Processing pitch (incoming raw_pitch is in range ~[-0.7, 0.7]). But need to take into account right_toggle's state first
        if raw_pitch < -0.7:  # right toggle is up (approx condition)
            processed_pitch = 0.0  # no pitch 
        else:
            processed_pitch = self.map_to_range(raw_pitch, 0.75, self.convert_to_rad(self.MAX_PITCH))
        
        p = Pose()
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = self.euler_to_quaternion(0.0, -processed_pitch, processed_yaw)
        self.body_pose_pub.publish(p)
        
if __name__ == '__main__':
    rospy.init_node('spot_pitch_yaw')
    spot_pitch_yaw = SpotPitchYaw()

    def signal_handler(sig, frame):
        print("Ctrl+C detected! Killing spot_pitch_yaw node...")
        rospy.sleep(2)
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    time.sleep(1)
    rospy.spin()
