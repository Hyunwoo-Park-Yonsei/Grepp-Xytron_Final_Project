#!/usr/bin/env python
#-*-coding:utf-8-*-
# 동영상 파일 읽기 (video_play.py)
import rospy, rospkg
import numpy as np
from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
import sys
import os
import signal
import math
# from numpy.lib.histograms import histogram
import cv2
from cv_bridge import CvBridge
import numpy as np
def traf_det(image):
    sign_roi,_,_ = set_roi_color(image,200,50,40)

    #img = cv2.medianBlur(sign_roi,5)
    hsv = cv2.cvtColor(sign_roi, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    _,v = cv2.threshold(v, 200,255,cv2.THRESH_BINARY)
    cimg = cv2.cvtColor(v,cv2.COLOR_GRAY2BGR)    
    circles = cv2.HoughCircles(v, cv2.HOUGH_GRADIENT, 1,10, param1=5, param2=5, minRadius=0, maxRadius=100)
    
    ##찾아진 서클을 하나하나씩 루프를 돌면서
    try:
        circles = np.uint16(np.around(circles))
        for i in circles[0,:]:
            cv2.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
            #i[0],i[1]은 원의 중심좌표 (x,y)
            #i[2]는 원의 반지름
            #따라서 (i[0],i[1])좌표에 반지름 길이i[2]으로 녹색으로 색칠한다.  두께는 2

            cv2.circle(cimg,(i[0],i[1]),2,(0,0,255),3)
            #중심은 빨간색으로 색칠
            #반지름이 2라서 매우 작으니까 점처럼 보이지만 원이다. 두께는 3
            cr_img = v[i[1]-10 : i[1]+10, i[0]-10 : i[0]+10]
            #원의 중심 좌표 -10에서 +10까지하면 한변 길이 20짜리 정사각형 생김
            #그 원의 v성분 값을 cr_img로 저장함. 
            img_str = 'x: {0}, y: {1}, mean : {2}'.format(i[0],i[1], cr_img.mean())
            #좌표값, v성분들의 평균값mean()을  구함
            #print(img_str)
            if  i[0] >50 and i[0] <56 :
                print "Red"
            elif i[0] > 70 and i[0] < 99:
                print "Yellow"
            elif i[0] >105 :
                print "Green"


    except AttributeError:
        pass


    return v

def setLabel(img,pts, label):
    (x,y,w,h) = cv2.boundingRect(pts)
    pt1 = (x,y)
    pt2 = (x+w, y+h)
    cv2.rectangle(img,pt1,pt2,(0,255,0),2)
    cv2.putText(img,label,(pt1[0],pt1[1]-3), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255))

## 방지턱 인식하기 
def bump_det(image) :
    
    sign_roi,_,_ = set_roi_color(image,200,200,80)
    H,L,S = cv2.split(cv2.cvtColor(sign_roi,cv2.COLOR_BGR2HLS))
    
    _,L = cv2.threshold(L,200,255, cv2.THRESH_BINARY)

    ##침식할 때 필요한 구조화 요소 커널 사각형 클수록 더 침식 잘됨.
    k = cv2.getStructuringElement(cv2.MORPH_RECT, (4,4))
    # 사각형 5x5
    eros = cv2.erode(L,k)
    

    _,contours,_ = cv2.findContours(eros, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    
    for cont in contours:
        approx = cv2.approxPolyDP(cont, cv2.arcLength(cont,True) *0.02, True)
        vtc = len(approx)
        if vtc ==6:
            setLabel(sign_roi, cont, 'bump')
            print "bump"
    return sign_roi



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
def set_roi_color(frame, x_len, start_y, offset_y):
    _,width,_ =  frame.shape
    #컬러 전용은 shape에서 리턴3개 나온다.  흑백은 리턴2개 나옴. 
    start_x = int(width/2 - (x_len/2))
    end_x = int(width -start_x)
    return frame[start_y:start_y+offset_y, start_x:end_x], start_x, start_y
if __name__== '__main__':
    video_file = "bump.mp4" # 동영상 파일 경로
    cap = cv2.VideoCapture(video_file) # 동영상 캡쳐 객체 생성  ---①
    ret,image = cap.read()
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
    if cap.isOpened():                 # 캡쳐 객체 초기화 확인
        while True:
            ret, image = cap.read()      # 다음 프레임 읽기      --- ②
            if ret:                     # 프레임 읽기 정상
                image = calibrate_image(image)

                #traf =traf_det(image)
                bump = bump_det(image)
                cv2.imshow(video_file, image) # 화면에 표시  --- ③
                cv2.imshow("traf", bump)
                cv2.waitKey(25)            # 25ms 지연(40fps로 가정)   --- ④
            else:                       # 다음 프레임 읽을 수 없슴,
              break                   # 재생 완료
    else:
        print("can't open video.")      # 캡쳐 객체 초기화 실패
    #cap.release()                       # 캡쳐 자원 반납
    #cv2.destroyAllWindows()
