#!/usr/bin/env python
# -*-coding:utf-8-*-

import numpy as np
import cv2

class StopDetect:
    frame = None
    Width = 640
    Height = 480

    def __init__(self):
        pass

    def setLabel(self, img, pts, label):
        (x, y, w, h) = cv2.boundingRect(pts)
        pt1 = (x, y)
        pt2 = (x+w, y+h)
        cv2.rectangle(img, pt1, pt2, (0,255,0), 2)
        cv2.putText(img, label, (pt1[0],pt1[1]-3), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255))
        return img

    def set_roi_color_stop(self, frame, x_len, start_y, offset_y):
        _,width,_ = frame.shape
        #컬러 전용은 shape에서 리턴3개 나온다.  흑백은 리턴2개 나옴. 
        start_x = int(width/2 - (x_len/2))
        end_x = int(width - start_x)
        return frame[start_y:start_y+offset_y, start_x-50:end_x-50], start_x, start_y

    def stopline_det(self, image):
        detected = False
        cont_min= 300
        cont_max = 0
        cont_width = 0
        sign_roi, _, _ = self.set_roi_color_stop(image,350,280,80)
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
                    img = self.setLabel(sign_roi, cont, 'stopline')
                    detected = True
                print ("contours   ")
        return img, detected
