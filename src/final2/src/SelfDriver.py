#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import time
import copy
import pickle
import cv2
import numpy as np
from matplotlib import pyplot as plt
import time

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
        self.CANNY_THRESHOLD_LOW = config.get("canny_threshold_low", 80)
        self.CANNY_THRESHOLD_HIGH = config.get("canny_threshold_high", 90)

        # 참고: cv2.getOptimalNewCameraMatrix
        # https://docs.opencv.org/3.3.0/dc/dbb/tutorial_py_calibration.html
        optimal_camera_matrix, optimal_camera_roi = cv2.getOptimalNewCameraMatrix(self.CAMERA_MATRIX, self.DISTORTION_COEFFS, (self.IMAGE_WIDTH, self.IMAGE_HEIGHT), 1, (self.IMAGE_WIDTH, self.IMAGE_HEIGHT))
        self.OPTIMAL_CAMERA_MATRIX = optimal_camera_matrix
        self.OPTIMAL_CAMERA_ROI = optimal_camera_roi

        # state definition
        # 0 끼어들기
        # 1 끼어들기 완료후 주행
        # 5 신호등
        # self.DrivingState = Enum("DrivingState", "none stop_line bump passenger")
        self.DRIVING_STATE_NONE = -1
        self.DRIVING_STATE_JOINT = 0
        self.DRIVING_STATE_NORMAL = 1
        self.DRIVING_STATE_TRAFFIC_LIGHT = 5
        self.DRIVING_STATE_STOP_LINE = 7
        self.DRIVING_STATE_BUMP = 8
        self.DRIVING_STATE_PASSENGER = 9
        self.DRIVING_STATE_ARPARKING = 10


        self.driving_state = 0



        # member variables
        self.sensor_data = None
        self.display_board = None
        self.image_helper = ImageHelper()
        self.lidar_helper = LidarHelper.LidarHelper()
        self.ultra_helper = UltraHelper()
        self.ar_helper = ArHelper()
        self.stop_detect = StopDetect()
        self.traffic_detect = TrafficDetect()
        self.bump_detect = BumpDetect()
        self.last_center = 300
        self.count = 0
        self.arNum = -1
        self.dist = -1
        self.start_time = 0
        self.lidar_front = 100
        self.cnt_right = 0
        self.last_time = time.time()

        self.yolo_stop_time = 0
        self.yolo_state = 0
        self.cat_detect_history = []
        self.people_detect_history = []

		self.parallel_parking_state = 0 
		self.parallel_parking_last_time = 0 
		self.parallel_distance_right_history = []

        pkl_file_name = "2021-09-14-pose_2-sampled-300.pkl"
        absolute_path = os.path.abspath(__file__)
        file_directory = os.path.dirname(absolute_path)
        pkl_file = os.path.join(file_directory, "../utils", pkl_file_name)

        with open(pkl_file, "rb") as f:
            self.path = pickle.load(f)

    def get_next_direction(self, sensor_data):

        # copy sensor deeply to make them synchronized throughout this function
        self.sensor_data = copy.deepcopy(sensor_data)

#        if self.sensor_data.pose is not None:
#            x = self.sensor_data.pose.position.x
#            y = self.sensor_data.pose.position.y
#            nearest_point_index = self.nearest_path_point(x, y)
#            print("{}: ({}, {})".format(nearest_point_index, self.path["x"][nearest_point_index], self.path["y"][nearest_point_index]))

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
            self.lidar_front = self.lidar_helper.lidar_front(self.sensor_data.ranges)
        else:
            print("no lidar_msg")

        if self.sensor_data.ar:
            self.arNum, self.dist = self.ar_helper.ArData(self.sensor_data.ar)
            print("arNum", self.arNum, "dist", self.dist)

        steer, speed = self.drive(warped, image_undistorted)

        # show ultra in display_board
        if self.sensor_data.ultra:
            image_size = (self.IMAGE_WIDTH, self.IMAGE_HEIGHT)
            display_ultra = self.ultra_helper.ultra_get(image_size, self.sensor_data.ultra)
            self.display_board = cv2.vconcat([self.display_board, display_ultra])
        else:
            print("no ultra_msg")

        return steer, speed


    def drive(self, image, image_undistorted):
        #cv2.imshow("image",image)
        print("state", self.driving_state)
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
            lpos, rpos = 350, 520

        #########
        if self.driving_state == 0:
            llpos = -1
            for i in range(lpos - 50,-1,-1):
                if image[200][i][0] > 0:
                    llpos = i
                    break
            print("llpos", llpos)
            if 130>lpos - llpos >110 and llpos != -1 and self.count > 10:
                rpos = lpos
                lpos = llpos
                print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
                self.driving_state = 1
                self.start_time = time.time()
                self.count = 0
        elif self.driving_state == 1 and self.sensor_data.ultra != None:
            print("ultra message")
            print(self.sensor_data.ultra[4], self.sensor_data.ultra[5])


            if self.sensor_data.ultra[4] <40 or self.sensor_data.ultra[5] < 40 and self.count > 10:
                print("!!!!!!!!!!!!!!!!!!!!!!!!!!")
                lpos, rpos = 330,480
                self.driving_state = 0
                self.count = 0
            if time.time() - start_time >15:
                self.driving_state = 2





        ##나중에 지워야함

#        print("state",state)
#        print(lpos,rpos)
#        print(rpos-lpos)
#        print(" ")
        #############

        self.last_center = (rpos+lpos) // 2
        steer = int((self.last_center-300) // 2)
        speed = 15


        if self.driving_state == 2:
            #img, stopline_detected = self.stop_detect.stopline_det(image_undistorted)
            #cv2.imshow("img",img)
            #if stopline_detected:
                #angle = 0
                #speed = 0
                #print("stop line detected")
            traffic_sign = self.traffic_detect.traf_det(image_undistorted)
            if not traffic_sign:
                img, stopline_detected = self.stop_detect.stopline_det(image_undistorted)
                if stopline_detected:
                    angle = 0
                    speed = 0
                    print("stop line detected")

            else:
                self.driving_state = 4
                self.start_time = time.time()



#        elif self.driving_state ==3:
#            if time.time()-self.start_time < 3:
#                speed = 15
#                angle = 0
#            elif time.time()-self.start_time < 6:
#                speed = 15
#                angle = 50
#            else:
#                self.driving_state = 4




        if self.arNum == 0 and self.dist < 0.6 and self.driving_state == 4:

            self.driving_state = 5
            self.start_time = time.time()
        elif self.driving_state == 5:
            t = 2.5
            if time.time() - self.start_time < t:
                speed = 15
                steer = 50
            elif time.time() - self.start_time < t + 1:
                speed = 0
                steeer = 0
            elif time.time() - self.start_time < 1.8*t + 1:
                speed = -20
                steer = -50
            elif time.time() - self.start_time < 2.3* t + 1:
                speed = -20
                steer = 50
            else:
                if self.dist > 0.8:
                    self.driving_state = 6
                    self.start_time = time.time()
                else:
                    speed = -20
                    steer = 0
        elif self.driving_state == 6:
            speed = 0
            steer = 0
            if 4 > time.time() - self.start_time > 3:
                speed = 15
                steer = -20
            elif 8> time.time() - self.start_time > 4:
                speed = 15
                steer = -40
            elif time.time() - self.start_time > 8:
                self.driving_state = 7
                self.last_center = 300

        elif self.driving_state == 7:
            # print("it is yolo state")

            if self.sensor_data.yolo_boxes is None:
                speed = 0

            else:
                len_history = 3
                # detect num of cats and num of people
                cats = len(self.find_cats())
                people = len(self.find_people())

                # insert detection result to history
                self.cat_detect_history.append(cats)
                self.cat_detect_history = self.cat_detect_history[-len_history:]

                self.people_detect_history.append(people)
                self.people_detect_history = self.people_detect_history[-len_history:]

                print(self.yolo_state)
                print(self.people_detect_history)

                if self.yolo_state == 0:
                    # 최근 3번의 yolo detection 결과가 모두 1개 이상일 때,
                    # 오브젝트가 확실히 있다고 판단
                    # 남자를 찾을 때까지 차선 주행
                    people_detected = filter(lambda x: x>0, self.people_detect_history)
                    if len(people_detected) == len_history:
                        self.yolo_state = 1
            
                elif self.yolo_state == 1:
                    # 최근 3번의 yolo detection 결과가 모두 0일 때,
                    # 오브젝트가 확실히 없다고 판단
                    # 정지 타이머 초기화
                    # print(self.people_detect_history)
                    people_detected = filter(lambda x: x>0, self.people_detect_history)
                    if len(people_detected) == 0:
                        self.yolo_stop_time = time.time()
                        self.yolo_state = 2
                        speed = 0

                elif self.yolo_state == 2:
                    # 1초간 정지
                    now = time.time()
                    delta_time = now - self.yolo_stop_time
                    if delta_time < 1:
                        speed = 0
                    else:
                        self.yolo_state = 3

                elif self.yolo_state == 3:
                    # 최근 3번의 yolo detection 결과가 모두 1개 이상일 때,
                    # 오브젝트가 확실히 있다고 판단
                    # 고양이를 찾을 때까지 차선 주행
                    cat_detected = filter(lambda x: x>0, self.cat_detect_history)
                    if len(cat_detected) == 3:
                        self.yolo_state = 4

                elif self.yolo_state == 4:
                    # 최근 3번의 yolo detection 결과가 모두 0일 때,
                    # 오브젝트가 확실히 없다고 판단
                    # 정지 타이머 초기화
                    cat_detected = filter(lambda x: x>0, self.cat_detect_history)
                    if len(cat_detected) == 0:
                        self.yolo_stop_time = time.time()
                        self.yolo_state = 5
                        speed = 0

                elif self.yolo_state == 5:
                    # 1초간 정지
                    now = time.time()
                    delta_time = now - self.yolo_stop_time
                    if delta_time < 1:
                        speed = 0
                    else:
                        self.yolo_state = 6

                elif self.yolo_state == 6:
                    # yolo_state 6 --> complete taxi mission
                    # s자 구간 통과까지 slam으로 위치 파악 및 bump detection state로 전환
                    x = self.sensor_data.pose.position.x
                    y = self.sensor_data.pose.position.y
                    nearest_point_index = self.nearest_path_point(x, y)
                    if 197 > nearest_point_index > 192:
                        self.driving_state = 8


        elif self.driving_state == 8:

            bump = self.bump_detect.bump_det(image_undistorted)
            if bump:
                self.driving_state = 9



        elif self.driving_state == 9:
            speed = 20
            steer = -2
            if self.lidar_front < 2.2:
                self.driving_state = 10
                self.start_time = time.time()
        elif self.driving_state == 10:
            speed = 0
            steer = -2
            self.cnt_right += len([x for x in self.sensor_data.ranges_right if 0 < x < 0.5])
            if self.cnt_right > 10 and time.time() - self.start_time > 2:
                self.driving_state = 11
                self.start_time = time.time()
                self.last_center = 300

        elif self.driving_state == 11:
            speed = 15
            if time.time() - self.start_time < 3:
                steer =20


        elif self.driving_state == 15 
			if self.sensor_data.ultra is None:
				speed = 10

			else:
                len_history = 5

				distance_right = self.sensor_data.ultra[4]
				distance_right_back = self.sensor_data.ultra[5]
				self.parallel_distance_right_history.append(distance_right)
				self.parallel_distance_right_history = self.parallel_distance_right_history[-len_history:]

				rights_over_50 = filter(lambda x: x>50, self.parallel_distance_right_history)
				print("rights history: {}".format(self.parallel_distance_right_history))

                if self.parallel_parking_state == 0:
					# if recent every five right distance values are all over 50 cm 
					# and right_back_distance is over 50 cm
					# try parking

					if len(rights_over_50) == len_history and distance_right_back > 50:
						# test this condition can be reached
						speed = 0
						self.parallel_parking_state = 1
						self.parallel_parking_last_time = time.time()

				elif self.parallel_parking_state == 1:
					now = time.time()
					time_delta = now - self.parallel_parking_last_time

					if time_delta < 3.5:
						# go straight
						speed = 15
						steer = 0

					if time_delta < 5:
						# drive to the right back
						speed = -20
						steer = 50

					elif time_delta < 6:
						# drive straight back
						speed = -20
						steer = 0

					elif time_delta < 7.5:
						# drive to the left back
						speed = -20
						steer = -50

					else:
						self.parallel_parking_state = 2

				elif self.parallel_parking_state == 2:
					# all mission are copleted!
					speed = 0
					steer = 0
						


        self.display_board = cv2.line(self.display_board,(self.last_center,445),(self.last_center,445),(255,0,0),30)
        self.display_board = cv2.line(self.display_board,(lpos,445),(lpos,445),(0,255,0),30)
        self.display_board = cv2.line(self.display_board,(rpos,445),(rpos,445),(0,255,0),30)



        self.count += 1
        return steer, speed

    def nearest_path_point(self, x, y):

        min_dist = 1e9
        min_index = 0
        n_points = len(self.path["x"])

#        front_x = x + L * np.cos(yaw)
#        front_y = y + L * np.sin(yaw)

        for i in range(n_points):
            # calculating distance (map_xs, map_ys) - (front_x, front_y)
            dx = x - self.path["x"][i]
            dy = y - self.path["y"][i]
            dist = np.hypot(dx, dy)

            if dist < min_dist:
                min_dist = dist
                min_index = i

        return min_index

    ###### Yolo
    def find_people(self):
        return self.find_yolo("person")

    def find_cats(self):
        return self.find_yolo("cat")

    def find_yolo(self, class_name):
        boxes = self.sensor_data.yolo_boxes
        ret = [box for box in boxes if box.Class == class_name]

        return ret
        # for i in range(len(boxes)):
        #     if
        #   yolo_data = [-1,-1,-1,-1]
        #   area = (boxes[i].xmax - boxes[i].xmin) * (boxes[i].ymax - boxes[i].ymin)
        #   #print(boxes[i].Class)
        #   if boxes[i].Class == "pottedplant" :
        #       yolo_data = ["pottedplant",boxes[i].xmin, boxes[i].xmax,area]
    ######


    def visualize(self):

        if self.display_board is not None:
            cv2.imshow("Sensor data display board", self.display_board)

        cv2.waitKey(1)
