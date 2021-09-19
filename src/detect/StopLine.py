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
        """
        도형찾고 라벨링
        """

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
            cont_xmin= 300
            cont_xmax = 0
            cont_xwidth = 0
            cont_ymin= 300
            cont_ymax = 0
            cont_ywidth = 0 
            if vtc == 2:
                for j in cont:
                  
                                
                    
                    i = j[0][0]
                    k = j[0][1]                    
                                        
                    if cont_xmax < i:
                        cont_xmax = i
                    if cont_xmin > i:
                        cont_xmin = i
                    if cont_ymax < k:
                        cont_ymax = k
                    if cont_ymin > k:
                        cont_ymin = k    
                                                                    

                cont_xwidth = cont_xmax -cont_xmin
                cont_ywidth = cont_ymax - cont_ymin
                print()                
                print("x, y width", cont_xwidth,cont_ywidth)                                
                print()
                if cont_xwidth > 179 and cont_ywidth < 50:
                    print("x, y width", cont_xwidth,cont_ywidth)                
                    img = self.setLabel(sign_roi, cont, 'stopline')
                    detected = True
        return img, detected
