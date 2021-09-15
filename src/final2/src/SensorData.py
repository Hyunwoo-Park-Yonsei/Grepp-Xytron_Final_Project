#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
from cv_bridge import CvBridge
from helpers.LidarHelper import LidarHelper

bridge = CvBridge()

class SensorData:
    def __init__(self):

        # self.image = np.empty(shape=[0])
        self.image = None
        self.ultra = None
        self.ranges = None
        self.ranges_left = None
        self.ranges_right = None
        self.ar = None

        self.lidar_helper = LidarHelper()

    def image_callback(self, msg):
        self.image = bridge.imgmsg_to_cv2(msg, "bgr8")

    def lidar_callback(self, msg):
        self.ranges = msg.ranges
        self.ranges_left = self.ranges[:self.lidar_helper.degree_to_lidar(90.)]
        self.ranges_right = self.ranges[self.lidar_helper.degree_to_lidar(270.):]


    def ultra_callback(self, msg):
        self.ultra = msg.data

    def ar_callback(self, msg):
        self.ar = msg.markers
