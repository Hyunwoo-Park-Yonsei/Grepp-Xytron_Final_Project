#!/usr/bin/env python
# -*-coding:utf-8-*-

import numpy as np
import cv2

class BumpDetect:
    def __init__(self):
        pass

    def setLabel(self, img, pts, label):
        (x,y,w,h) = cv2.boundingRect(pts)
        pt1 = (x,y)
        pt2 = (x+w, y+h)
        cv2.rectangle(img,pt1,pt2,(0,255,0),2)
        cv2.putText(img,label,(pt1[0],pt1[1]-3), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255))

    def set_roi_color_bump(self, frame, x_len, start_y, offset_y):
        _,width,_ =  frame.shape
        #컬러 전용은 shape에서 리턴3개 나온다.  흑백은 리턴2개 나옴. 
        start_x = int(width/2 - (x_len/2))
        end_x = int(width -start_x)
        return frame[start_y:start_y+offset_y, start_x-50:end_x-50], start_x, start_y

    def bump_det(self, image):
        """
        방지턱 인식하기 
        """

        sign_roi,_,_ = self.set_roi_color_bump(image,250,230,80)
        H,L,S = cv2.split(cv2.cvtColor(sign_roi,cv2.COLOR_BGR2HLS))

        _,L = cv2.threshold(L,200,255, cv2.THRESH_BINARY)
        #print (L)

        ##침식할 때 필요한 구조화 요소 커널 사각형 클수록 더 침식 잘됨.
        k = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
        # 사각형 5x5
        eros = cv2.erode(L,k)


        _,contours,_ = cv2.findContours(eros, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        for cont in contours:
            approx = cv2.approxPolyDP(cont, cv2.arcLength(cont,True) *0.02, True)
            vtc = len(approx)
            if vtc ==6:
                self.setLabel(sign_roi, cont, 'bump')
                print "bump"
        return sign_roi
    
