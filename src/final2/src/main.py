#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os
import signal

import rospy, rospkg
import numpy as np
from SensorData import SensorData
from SelfDriver import SelfDriver

from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32MultiArray

def signal_handler(sig, frame):
    os.system("killall -9 python rosout")
    sys.exit(0)


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)

    driver_config = {
        "image_width": 640,
        "image_height": 480,
        "image_offset": 280,
        "image_gap": 36,
        "lane_bin_threshold": 130,
        "camera_matrix": np.array([
            [422.037858, 0.0, 245.895397],
            [0.0,435.589734, 163.625535],
            [0.0,0.0, 1.0]
        ]),
        "distortion_coeffs": np.array([-0.2789296, 0.061035, 0.001786, 0.015238, 0.0]),
    }

    driver = SelfDriver(driver_config)
    sensor_data = SensorData()

    motor_msg = xycar_motor()
    pub = rospy.Publisher("xycar_motor",xycar_motor, queue_size=1)

    rospy.init_node("lane_detect")
    rospy.Subscriber("/usb_cam/image_raw", Image, sensor_data.image_callback, queue_size=1)
    rospy.Subscriber("/scan", LaserScan, sensor_data.lidar_callback, queue_size=1)
    rospy.Subscriber("xycar_ultrasonic", Int32MultiArray, sensor_data.ultra_callback, queue_size=1)

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        # main()
        steer, speed = driver.get_next_direction(sensor_data)
        motor_msg.angle = steer
        motor_msg.speed = speed

        pub.publish(motor_msg)

        # driver.visualize()

        rate.sleep()
