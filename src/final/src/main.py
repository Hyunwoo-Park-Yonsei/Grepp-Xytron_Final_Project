#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy, rospkg
import numpy as np
from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
import sys
import os
import signal
import math
from matplotlib import pyplot as plt

# from numpy.lib.histograms import histogram
import cv2
from cv_bridge import CvBridge
import numpy as np

from detect.Bump import BumpDetect
from detect.TrafficLight import TrafficDetect
from detect.StopLine import StopDetect


# 동영상으로 테스트인 경우, 실차에서는 제거
#cap = cv2.VideoCapture("/video/xycar_track1.mp4")

def callback_Lidar(data):
    global s, left_sensor, right_sensor
    s = data.ranges
    ratio = 1.4027
    left_sensor = []
    right_sensor = []
    for i in range(int(90*ratio)):
        left_sensor.append(s[i])
    for i in range(int(270*ratio),int(360*ratio)):
        right_sensor.append(s[i])



def img_callback(data):
    global image, bridge
    image = bridge.imgmsg_to_cv2(data,"bgr8")

def lidar_visualizer(img,left_sensor,right_sensor):
    ratio = 1.4027
    sensors = right_sensor+left_sensor
    interface = img

    for i,sensor in enumerate(sensors):
        angle = (i / ratio)*np.pi / 180
        sensor = sensor*100
        #print("i", i, "sensor",sensor,'angle',angle)
        scale = 3.5
        point = (320+int(sensor*np.cos(angle)*scale),480-int(sensor*np.sin(angle)*scale))
        #print('point',point)
        interface = cv2.line(interface,point,point,(255,0,0),10)

    return interface

def warp_image(img):
    #pts1 =np.float32([[228,290],[75,385],[423,290],[573,385]])
    #pts2 =np.float32([[228,290],[228,480],[423,290],[423,480]])

    pts1 =np.float32([[216,245],[34,358],[380,245],[535,358]])
    pts2 =np.float32([[216,245],[216,480],[380,245],[380,480]])

    M= cv2.getPerspectiveTransform(pts1,pts2)
    Minv = cv2.getPerspectiveTransform(pts2,pts1)

    size = (640, 480)
    dst =cv2.warpPerspective(img, M, size,  flags=cv2.INTER_LINEAR)
    _, dst2 = cv2.threshold(dst,lane_bin_th, 255, cv2.THRESH_BINARY)
    return Minv, dst2
##자이카 카메라로 촬영한 동영상이므로 전용 보정값 써야햔다.


## 이미지 한장 읽을 때마다 위에서 구한 보정 행렬값을 적용하여 이미지를 반듯하게 수정하는 함수
def calibrate_image(frame):
    global Width, Height
    global mtx, dist
    global cal_mtx, cal_roi

    tf_image = cv2.undistort(frame, mtx, dist, None, cal_mtx)
    #보정행렬값을 적용하여 반듯하게 수정하는 함수
    x,y,w,h = cal_roi
    tf_image = tf_image[y:y+h, x:x+w]

    return cv2.resize(tf_image, (Width, Height))

##변환 전과 후의 4개 점 좌표를 전달해서 이미지를 원근 변환처리한다.

def signal_handler(sig, frame):
    os.system('killall -9 python rosout')
    sys.exit(0)


def img_processing(img):
    img = calibrate_image(img)
    blur = cv2.GaussianBlur(img,(5,5),0)
    
    #cv2.imshow('raw',img)


    LANE_MIN = np.array([20,5 , 55],np.uint8)
    LANE_MAX = np.array([155, 105, 140],np.uint8)

    hsv_img = cv2.cvtColor(blur,cv2.COLOR_BGR2HSV)

    lane_img = cv2.inRange(hsv_img, LANE_MIN, LANE_MAX)
    kernel = np.ones((5,5),np.uint8)
    dila = cv2.dilate(lane_img,kernel)

  
    #cv2.imshow('dila',dila)

    return dila, img


# 검게 보이는 부분 하얗게 날리기
def black2white(image):
    width = 640
    height = 480

    # 좌삼각
    point = np.array([[0, 235], [0, height], [204, height]], np.int32)
    image = cv2.fillConvexPoly(image, point, 255)
    # 우삼각
    point = np.array([[415, height], [width, height], [width, 235]], np.int32)
    image = cv2.fillConvexPoly(image, point, 255)

    # return image
    
    # crop
    crop_img = image[200:450, 120:480]
    return crop_img

def waypoint(image):

    waypoints = []
    # cv2.imshow('image',image)
    lpos, rpos = -1,-1
    for i in range(320,640):
        if image[460][i][0] > 0:
            rpos = i
    for i in range(320,-1,-1):
        if image[460][i][0] > 0:
            lpos = i
    
    
    new_img = cv2.line(image,(0,460),(640,460), (0,0,255), 2)
    image.mean(axis=2)
    
    #480,640,3
    c = (rpos+lpos)//2
    steer = int(c -320)
    speed = 15
    
    
    new_img = cv2.line(image,(c,460),(c,460),(255,255,0),5)
    
    # cv2.imshow('new_img', new_img)
    return waypoints

def main():
    global image, cap, s

    # state 0 : 정지선 검출 중
    state_0 = True

    # state 1 : 방지턱 검출 중
    state_1 = False

    # 동영상으로 테스트인 경우, 실차에서는 제거
    # _, image = cap.read()

    # 사진으로 테스트
    # image = cv2.imread("img/raw2.png", cv2.IMREAD_ANYCOLOR)
    
    while True:

        while not image.size == (640 * 480 * 3):
            continue

        #print(1)
        gray, cali_img = img_processing(image)
        # cv2.imshow('cali_img', cali_img)
        #print('gray',gray.shape)


        # state_0 : 정지선
        # state_1 : 방지턱 검출
        if state_0 == True and state_1 == False:
            # 방지턱을 찾지 않을 떄는 정지선 검출
            stop_detect = StopDetect(cali_img)
            stop_detect.detect_stopline()
            if stop_detect.signal == True: # 정지선 검출
                state_0 = False # 정지선 더 찾지 않음
                state_1 = True # 방지턱 찾기 시작
                print("stopline")
                # 정지선 정지 후 신호등 검출
                # 신호등 이미지 받아오기

                # 신호등 판별
                # traffic_detect = TrafficDetect(cali_img)
                # traffic_detect.traf_det()
                # if traffic_detect.signal == "Red":
                #     print("red")
                # elif traffic_detect.signal == "Yellow":
                #     print("yelow")
                # elif traffic_detect.signal == "Green":
                #     print("green")
                # else:
                #     pass

            # else:
            #     pass

        # state_0 : 정지선
        # state_1 : 방지턱 검출
        if state_0 == False and state_1 == True:
            # 방지턱 검출
            bump_detect = BumpDetect(cali_img)
            bump_detect.bump_det()
            if bump_detect.find == True: # 방지턱 검출
                state_0 = True
                state_1 = False
                print("bump")
            else:
                pass

        


        Minv, warped = warp_image(gray)
        # cv2.imshow('warped2', warped)
        warped = cv2.cvtColor(warped,cv2.COLOR_GRAY2BGR)
            
        #print('warped',warped.shape)

        # 차선 검출 전에 이미지 전처리하기
        # print(warped[320-1][640-1])
        ##warped = black2white(warped)
        #cv2.imshow('cut and warped', warped)
      
        interface = lidar_visualizer(warped,left_sensor,right_sensor)
        #cv2.imshow('interface',interface)
        waypoints = waypoint(warped)
        
        cv2.waitKey(1)


if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    image = np.empty(shape=[0])
    bridge = CvBridge()
    pub = rospy.Publisher('xycar_motor',xycar_motor)
    Width = 640
    Height = 480
    Offset = 280
    Gap = 36
    lane_bin_th = 130
    calibrated = True
    s = []
    left_sensor = []
    right_sensor = []
    if calibrated:
        mtx = np.array([
            [422.037858, 0.0, 245.895397],
            [0.0,435.589734, 163.625535],
            [0.0,0.0, 1.0]
        ])
        dist = np.array([-0.2789296, 0.061035, 0.001786, 0.015238, 0.0])
    
        cal_mtx, cal_roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (Width, Height),1,
                        (Width, Height))

    rospy.init_node('lane_detect')
    rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)
    rospy.Subscriber("/scan", LaserScan, callback_Lidar, queue_size=1)

    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        main()
        rate.sleep()


