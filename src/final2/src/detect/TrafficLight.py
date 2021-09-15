#!/usr/bin/env python
# -*-coding:utf-8-*-

import numpy as np
import cv2

class TrafficDetect:
    frame = None
    
    def __init__(self):
        pass
    
    def set_roi_color_traf(self, frame, x_len, start_y, offset_y):
        _, width, _ = frame.shape
        #컬러 전용은 shape에서 리턴3개 나온다.  흑백은 리턴2개 나옴. 
        start_x = int(width/2 - (x_len/2))
        end_x = int(width - start_x)
        return frame[start_y:start_y+offset_y, start_x:end_x], start_x, start_y

    def traf_det(self, image):
        """
        신호등 구분
        """

        sign_roi, _, _ = self.set_roi_color_traf(image, 200, 80, 100)
        #img = cv2.medianBlur(sign_roi,5)
        hsv = cv2.cvtColor(sign_roi, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)
        _, v = cv2.threshold(v, 220, 255, cv2.THRESH_BINARY)
        cimg = cv2.cvtColor(v, cv2.COLOR_GRAY2BGR)    
        circles = cv2.HoughCircles(v, cv2.HOUGH_GRADIENT, 1, 20, param1=180, param2=10, minRadius=5, maxRadius=15)
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
