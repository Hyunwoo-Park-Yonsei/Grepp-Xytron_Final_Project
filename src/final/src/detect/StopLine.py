#!/usr/bin/env python
# -*-coding:utf-8-*-

import numpy as np
import cv2

class StopDetect:
    frame = None
    Width = 640
    Height = 480

    def __init__(self, frame):
        self.frame = frame
        self.signal = None

    def set_roi(self, x_len, start_y, offset_y):
        _,width,_ = self.frame.shape
        start_x = int(width/2 - (x_len/2))
        end_x = int(width -start_x)
        return self.frame[start_y:start_y+offset_y, start_x:end_x], start_x, start_y

    ##정지선 인식하기
    def detect_stopline(self):
        stopline_roi, _, _ = self.set_roi(300, 280,25)
        #_,roi_inv = cv2.threshold(stopline_roi,127,255,cv2.THRESH_BINARY_INV)
        ##픽셀 갯수세기.
        cntPixels = self.Width * self.Height
        stopline_roi = cv2.cvtColor(stopline_roi, cv2.COLOR_BGR2GRAY)
        # cv2.imshow("stop_line", stopline_roi)
        
        cnt_white = cv2.countNonZero(stopline_roi)
        # print (cnt_white)
        cnt_black = cntPixels - cnt_white
        #roi로 범위 자르기
        ##명도차로 검은색 점 1000개 이상이면 정지선이다.
        # print("stopline_cnt")
        print(cnt_black)
        if cnt_black > 29000:
            print("stop!")
            self.signal = True
        else :
            print("not stop")
            self.signal = False
        return stopline_roi
