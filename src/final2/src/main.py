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
from std_msgs.msg import Int32MultiArray
# from numpy.lib.histograms import histogram
import cv2
from cv_bridge import CvBridge
import numpy as np
import time
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
##변환 전과 후의 4개 점 좌표를 전달해서 이미지를 원근 변환처리한다
def signal_handler(sig, frame):
    os.system('killall -9 python rosout')
    sys.exit(0)
def img_processing(img):
    img = calibrate_image(img)
    blur = cv2.GaussianBlur(img,(9,9),0)
    #cv2.imshow('raw',img)
    #LANE_MIN = np.array([100//2-20,0 , 13*255//100-5],np.uint8)
    #LANE_MAX = np.array([211//2+20, 255, 46*255//100+5],np.uint8)
    #hsv_img = cv2.cvtColor(blur,cv2.COLOR_BGR2HSV)
    low_threshold = 60
    high_threshold = 70
    edge_img = cv2.Canny(np.uint8(blur), low_threshold, high_threshold)
    kernel = np.ones((4,4),np.uint8)
    dila = cv2.dilate(edge_img,kernel)  
#    k = cv2.getStructuringElement(cv2.MORPH_RECT,(4,4))
#    eros = cv2.erode(dila,k)
    #cv2.imshow('dila',dila)
    return dila, img
def set_roi_color_traf(frame, x_len, start_y, offset_y):
    _,width,_ =  frame.shape
    #컬러 전용은 shape에서 리턴3개 나온다.  흑백은 리턴2개 나옴. 
    start_x = int(width/2 - (x_len/2))
    end_x = int(width -start_x)
    return frame[start_y:start_y+offset_y, start_x:end_x], start_x, start_y
##신호등 구분
def traf_det(image):
    sign_roi,_,_ = set_roi_color_traf(image,200,80,100)
    #img = cv2.medianBlur(sign_roi,5)
    hsv = cv2.cvtColor(sign_roi, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    _,v = cv2.threshold(v, 230,255,cv2.THRESH_BINARY)
    cimg = cv2.cvtColor(v,cv2.COLOR_GRAY2BGR)    
    circles = cv2.HoughCircles(v, cv2.HOUGH_GRADIENT, 1,20, param1=180, param2=10, minRadius=5, maxRadius=15)
    ##찾아진 서클을 하나하나씩 루프
   
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
            if 40> i[0] >20:
                print "Red"
            elif 60 >i[0] > 45 and i[0] < 99:
                print "Yellow"
            elif 90 >i[0] >70 :
                print "Green"     
    except AttributeError:
        pass
    return cimg
# 검게 보이는 부분 하얗게 날리기
def setLabel(img,pts, label):
    (x,y,w,h) = cv2.boundingRect(pts)
    pt1 =(x,y)
    pt2 = (x+w,y+h)
    cv2.rectangle(img,pt1,pt2,(0,255,0),2)
    cv2.putText(img,label,(pt1[0],pt1[1]-3), cv2.FONT_HERSHEY_SIMPLEX, 0.7,(0,0,255))
    return img
def set_roi_color_stop(frame, x_len, start_y, offset_y):
    _,width,_ =  frame.shape
    #컬러 전용은 shape에서 리턴3개 나온다.  흑백은 리턴2개 나옴. 
    start_x = int(width/2 - (x_len/2))
    end_x = int(width -start_x)
    return frame[start_y:start_y+offset_y, start_x-50:end_x-50], start_x, start_y
def stopline_det(image):
    detected = False
    cont_min= 300
    cont_max = 0
    cont_width = 0
    sign_roi,_,_ = set_roi_color_stop(image,350,280,80)
    gray = cv2.cvtColor(sign_roi, cv2.COLOR_BGR2GRAY)
    gray = 255- gray
    k = cv2.getStructuringElement(cv2.MORPH_RECT,(2,2))
    gray = cv2.erode(gray, k)
    ret, thr = cv2.threshold(gray, 0,255,cv2.THRESH_OTSU)
    _,contours, _ =cv2.findContours(thr,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    #cv2.imshow('thr',thr)
    img = thr
    for cont in contours:
        approx = cv2.approxPolyDP(cont, cv2.arcLength(cont,True) *0.02, True)
        vtc = len(approx)
        if vtc == 2:
            for j in cont:
                i = j[0][0]
                if cont_max < i:
                    cont_max = i
                if cont_min > i:
                    cont_min = i
            print("cont width" , cont_max - cont_min)
            cont_width = cont_max -cont_min
            if cont_width > 150:
                img = setLabel(sign_roi, cont, 'stopline')
                detected = True
            print ("contours   ")
    return img, detected
def ultra_call_back(data):
    global ultra_msg
    callback_time = time.time()
    ultra_msg = data.data
def Drive(image, cali_img):
    global c, state,ultra_msg, count
    #cv2.imshow('image',image)
    lpos, rpos = -1,-1
    for i in range(c,640):
        if image[445][i][0] > 0:
            rpos = i
            break
    for i in range(c,-1,-1):
        if image[445][i][0] > 0:
            lpos = i
            break
    if lpos ==-1 or rpos == -1:
        if rpos != -1:
            lpos = rpos -130
        elif lpos != -1:
            rpos = lpos + 130
        else:
            for i in range(c,640):
                if image[450][i][0] > 0:
                    rpos = i
                    break
            else:
                rpos = 640            
            for i in range(c,-1,-1):
                if image[450][i][0] > 0:
                    lpos = i
                    break
            else:
                lpos = 0
    new_img = cv2.line(image,(0,445),(640,445), (0,0,255), 2)
    image.mean(axis=2)
    #480,640,3
    if rpos - lpos < 10:
        lpos, rpos = 220, 380
    elif rpos -lpos < 100:
        rpos = lpos + 130
    elif rpos - lpos > 600:
        lpos, rpos = 350, 500
    if state == 0:
        llpos = -1
        for i in range(lpos - 50,-1,-1):
            if image[200][i][0] > 0:
                llpos = i
                break
        print("llpos", llpos)
        if 130>lpos - llpos >110 and llpos != -1 and count > 10:
            rpos = lpos
            lpos = llpos
            print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
            state = 1
            count = 0
    elif state == 1 and ultra_msg != None:
        print("ultra message")
        print(ultra_msg[4],ultra_msg[5])
        img, stopline_detected = stopline_det(cali_img)
        cv2.imshwo("trrf",traf)
        cv2.imshow("img", img)
        if stopline_detected:
            motor_msg.speed = 0
            motor_msg.angle = 0
            pub.publish(motor_msg)
            #time.sleep(5)
            #state = 2
            print("stop line detected")
        if ultra_msg[4] <40 or ultra_msg[5] < 40 and count > 10:
            print("!!!!!!!!!!!!!!!!!!!!!!!!!!")
            lpos, rpos = 330,480
            state = 0
            count = 0
    ## 신호등 모드 state = 5
    elif state == 5 and ultra_msg !=None:
        traf = traf_det(cali_img)
        cv2.imshow("traf",traf)
    ##나중에 지워야함
    elif state == 2:
        img, stopline_detected = stopline_det(cali_img)
        cv2.imshow("img", img)
        if stopline_detected:
            motor_msg.speed = 0
            motor_msg.angle = 0
            pub.publish(motor_msg)
            time.sleep(3)
            state = 3
            print("stop line detected")
    #############
    print("state",state)
    print(lpos,rpos)
    print(rpos-lpos)
    print(" ")
    c = (rpos+lpos)//2
    steer = int((c -300)//2)
    speed = 15
    motor_msg.angle = steer
    motor_msg.speed = speed
    pub.publish(motor_msg)
    new_img = cv2.line(image,(c,445),(c,445),(255,255,0),30)
    new_img = cv2.line(image,(lpos,445),(lpos,445),(0,255,0),30)
    new_img = cv2.line(image,(rpos,445),(rpos,445),(0,255,0),30)
    count +=1 
    #cv2.imshow('new_img', new_img)
def ultra_get():
    global ultra_msg
    zeros = np.zeros((480,640,3),np.uint8)
    # 초음파센서 데이터 출력
    #print("ultra_msg:", ultra_msg)
    side_right = ultra_msg[0]
    side_left = ultra_msg[4]
    rear_right = ultra_msg[7]
    rear_center = ultra_msg[6]
    rear_left = ultra_msg[5]
    #time.sleep(0.5)
    # 천천히 출력
    # 다음에 그래픽 만들기
    scale = 6
    scale_sin60 = int(scale * 0.5)
    scale_cos60 = int(scale  * 0.8)
    ##640의 절반인 320이 카메라 가운데가 아니라 300
    zeros = cv2.circle(zeros,(300-side_left*scale,10),6,(100,255,200),20)
    zeros = cv2.circle(zeros,(300+side_right*scale,10),6,(100,255,200),20)
    zeros = cv2.circle(zeros,(300+rear_right*scale_cos60,rear_right*scale_sin60),6,(100,255,200),20)
    zeros = cv2.circle(zeros,(300-rear_left*scale_cos60,rear_left*scale_sin60),6,(100,255,200),20)
    zeros = cv2.circle(zeros,(300,rear_center*scale),6,(100,255,200),20)
    zeros = cv2.circle(zeros,(300,-50),500,(0,0,255),5)
    #한계선
    return zeros

def main():
    global image, cap, s, ultra_msg, state
    # state
    # 0 끼어들기
    # 1 끼어들기 완료후 주행
    # 2 정지선 이후
    # 5 신호등
    state = 5
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
        #ultra_msg = None
        # state_0 : 정지선
        # state_1 : 방지턱 검출
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
        Drive(warped,cali_img)
        if ultra_msg == None:
            print("no ultra_msg")
            continue
        interface_ultra = ultra_get()
        concat = cv2.vconcat([interface, interface_ultra])
        cv2.imshow("final interface", concat)
        cv2.waitKey(1)
if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    image = np.empty(shape=[0])
    bridge = CvBridge()
    pub = rospy.Publisher('xycar_motor',xycar_motor, queue_size=1)
    Width = 640
    Height = 480
    Offset = 280
    Gap = 36
    lane_bin_th = 130
    calibrated = True
    s = []
    c = 300
    ultra_msg = None
    motor_msg = xycar_motor()
    left_sensor = []
    right_sensor = []
    count = 0
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
    rospy.Subscriber("/usb_cam/image_raw", Image, img_callback, queue_size=1)
    rospy.Subscriber("/scan", LaserScan, callback_Lidar, queue_size=1)
    rospy.Subscriber('xycar_ultrasonic', Int32MultiArray, ultra_call_back, queue_size=1)
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        main()
        rate.sleep()