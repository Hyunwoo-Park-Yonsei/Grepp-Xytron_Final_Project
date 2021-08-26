#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy, rospkg
import numpy as np
from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import Image
import sys
import os
import signal
import cv2
from cv_bridge import CvBridge
import cv2


def warp_image(img):
    pts1 =np.float32([[290,228],[385,75],[290,423],[385,573]])
    pts2 =np.float32([[290,228],[385,228],[290,423],[385,423]])


  
    M= cv2.getPerspectiveTransform(pts1,pts2)
    dst =cv2.warpPerspective(img,M, (400,300))
    return dst
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




def img_callback(data):
    global image, bridge
    image = bridge.imgmsg_to_cv2(data,"bgr8")

def img_processing(img):
    img = calibrate_image(img)
    cv2.circle(img, (290,228), 20, 0,-1)
    cv2.circle(img,(385,75), 20, 0,-1)
    cv2.circle(img,(290,423), 20, 0,-1)
    cv2.circle(img, (3385,573), 20,0,-1)
    cv2.imshow('point_raw', img)
    blur = cv2.GaussianBlur(img,(5,5),0)
    _,L,_ = cv2.split(cv2.cvtColor(blur,cv2.COLOR_BGR2HLS))
    _, gray = cv2.threshold(L,lane_bin_th, 255, cv2.THRESH_BINARY)
    
    kernel = np.ones((1,1),np.uint8)
    dila = cv2.dilate(gray,kernel)

    # 255, 385 ROI ths value
    

    #dila = cv2.line(dila, (0,290),(640,290), 0)
    #dila = cv2.line(dila, (0,385),(640,385), 0)
    #dila = cv2.circle(dila,(423,290),10,3)
    return dila

def main():
    global image
    while True:
        while not image.size == (640 * 480 * 3):
            continue
        cv2.imshow('raw', image)
        gray = img_processing(image)
        
        cv2.imshow('lane', gray)
        warped = warp_image(gray)
        cv2.imshow('warped', warped)
        if cv2.waitKey(5) & 0xFF == ord('q'):
            break


if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    image = np.empty(shape=[0])
    bridge = CvBridge()
    pub = None
    Width = 640
    Height = 480
    Offset = 280
    Gap = 36
    lane_bin_th = 130
    
    calibrated = True
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
    iamge_sub = rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)
    main()
    rospy.spin()

