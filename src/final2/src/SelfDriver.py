#!/usr/bin/env python
# -*- coding: utf-8 -*-

import copy
import cv2
import numpy as np
from matplotlib import pyplot as plt

from detect.Bump import BumpDetect
from detect.TrafficLight import TrafficDetect
from detect.StopLine import StopDetect

# from SensorData import SensorData
from helpers import *

class SelfDriver:
    def __init__(self, config):

        # constant variables
        self.IMAGE_WIDTH = config.get("image_width")
        self.IMAGE_HEIGHT = config.get("image_height")
        self.IMAGE_OFFSET = config.get("image_offset")
        self.IMAGE_GAP = config.get("image_gap")
        self.LANE_BIN_THRESHOLD = config.get("lane_bin_threshold")
        self.CAMERA_MATRIX = config.get("camera_matrix")
        self.DISTORTION_COEFFS = config.get("distortion_coeffs")
        self.CANNY_THRESHOLD_LOW = config.get("canny_threshold_low", 60)
        self.CANNY_THRESHOLD_HIGH = config.get("canny_threshold_high", 70)

        # 참고: cv2.getOptimalNewCameraMatrix
        # https://docs.opencv.org/3.3.0/dc/dbb/tutorial_py_calibration.html
        optimal_camera_matrix, optimal_camera_roi = cv2.getOptimalNewCameraMatrix(self.CAMERA_MATRIX, self.DISTORTION_COEFFS, (self.IMAGE_WIDTH, self.IMAGE_HEIGHT), 1, (self.IMAGE_WIDTH, self.IMAGE_HEIGHT))
        self.OPTIMAL_CAMERA_MATRIX = optimal_camera_matrix
        self.OPTIMAL_CAMERA_ROI = optimal_camera_roi

        # state definition
        # self.DrivingState = Enum("DrivingState", "none stop_line bump passenger")
        self.DRIVING_STATE_NONE = 0
        self.DRIVING_STATE_STOP_LINE = 1
        self.DRIVING_STATE_BUMP = 2
        self.DRIVING_STATE_PASSENGER = 3
        self.driving_state = self.DRIVING_STATE_NONE

        # member variables
        self.sensor_data = None
        self.display_board = None
        self.image_helper = ImageHelper()
        self.lidar_helper = LidarHelper.LidarHelper()
        self.ultra_helper = UltraHelper()
        self.last_center = 300

    def get_next_direction(self, sensor_data):

        # copy sensor deeply to make them synchronized throughout this function
        self.sensor_data = copy.deepcopy(sensor_data)

        # check correct image size
        if self.sensor_data.image is None:
            return 0, 0
        if not self.sensor_data.image.size == (640 * 480 * 3):
            return 0, 0

        image_size = (self.IMAGE_WIDTH, self.IMAGE_HEIGHT)
        image_dilated, image_undistorted = self.image_helper.img_processing(self.sensor_data.image, self.CAMERA_MATRIX, self.DISTORTION_COEFFS, self.OPTIMAL_CAMERA_MATRIX, self.OPTIMAL_CAMERA_ROI, image_size, self.CANNY_THRESHOLD_LOW, self.CANNY_THRESHOLD_HIGH)

        # perspective tranform
        Minv, warped = self.image_helper.warp_image(image_dilated, self.LANE_BIN_THRESHOLD)
        # cv2.imshow('warped2', warped)
        warped = cv2.cvtColor(warped,cv2.COLOR_GRAY2BGR)

        # show lidar in display_board
        if self.sensor_data.ranges:
            display_lidar = self.lidar_helper.lidar_visualizer(warped, self.sensor_data.ranges_left, self.sensor_data.ranges_right)
            self.display_board = display_lidar
        else:
            print("no lidar_msg")

        steer, speed = self.drive(warped)

        # show ultra in display_board
        if self.sensor_data.ultra:
            image_size = (self.IMAGE_WIDTH, self.IMAGE_HEIGHT)
            display_ultra = self.ultra_helper.ultra_get(image_size, self.sensor_data.ultra)
            self.display_board = cv2.vconcat([self.display_board, display_ultra])
        else:
            print("no ultra_msg")

        return steer, speed


    def drive(self, image):
        #cv2.imshow("image",image)
        lpos, rpos = -1, -1
        for i in range(self.last_center, 640):
            if image[445][i][0] > 0:
                rpos = i
                break
        for i in range(self.last_center, -1, -1):
            if image[445][i][0] > 0:
                lpos = i
                break

        if lpos == -1 or rpos == -1:
            if rpos != -1:
                lpos = rpos -130
            elif lpos != -1:
                rpos = lpos + 130
            else:
                for i in range(self.last_center, 640):
                    if image[450][i][0] > 0:
                        rpos = i
                        break
                else:
                    rpos = 640

                for i in range(self.last_center, -1, -1):
                    if image[450][i][0] > 0:
                        lpos = i
                        break
                else:
                    lpos = 0


        # new_img = cv2.line(image,(0,445),(640,445), (0,0,255), 2)
        image.mean(axis=2)

        #480,640,3

        if rpos - lpos < 10:
            lpos, rpos = 220, 380

        elif rpos -lpos < 100:
            rpos = lpos + 130

        elif rpos - lpos > 600:
            lpos, rpos = 350, 500


        self.last_center = (rpos+lpos) // 2
        steer = int((self.last_center-300) // 2)
        speed = 15

        # motor_msg.angle = steer
        # motor_msg.speed = speed
        # pub.publish(motor_msg)

        # new_img = cv2.line(image,(c,445),(c,445),(255,255,0),30)
        # new_img = cv2.line(image,(lpos,445),(lpos,445),(0,255,0),30)
        # new_img = cv2.line(image,(rpos,445),(rpos,445),(0,255,0),30)

        return steer, speed


    def visualize(self):

        if self.display_board is not None:
            cv2.imshow("Sensor data display board", self.display_board)

        cv2.waitKey(1)
