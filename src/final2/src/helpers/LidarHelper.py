#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np

class LidarHelper:
    def __init__(self):
        # constants
        self.DEGREE_TO_LIDAR_RATIO = 1.4027

    def degree_to_lidar(self, degree):
        return int(degree * self.DEGREE_TO_LIDAR_RATIO)

    def lidar_to_degree(self, index):
        return (index / self.DEGREE_TO_LIDAR_RATIO) * np.pi / 180

    def lidar_visualizer(self, img, left_sensor, right_sensor):
        sensors = right_sensor + left_sensor
        display_lidar = img

        for i, sensor in enumerate(sensors):
            angle = self.lidar_to_degree(i)
            sensor = sensor * 100
            # print("i", i, "sensor", sensor, "angle", angle)
            scale = 3.5
            point = (320 + int(sensor*np.cos(angle)*scale), 480 - int(sensor*np.sin(angle)*scale))
            # print("point", point)
            display_lidar = cv2.line(display_lidar, point, point, (255,0,0), 10)

        return display_lidar

    def lidar_front(self, sensors):
        total = 0
        count = 0
        for i in range(8):
            if sensors[i] != 0:
                count +=1
                total += sensors[i]
            if sensors[-i] != 0:
                count ==1
                total += sensors[i]
        try:
            return total/count
        except:
            return 100