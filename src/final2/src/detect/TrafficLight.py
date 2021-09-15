#!/usr/bin/env python
# -*-coding:utf-8-*-

import numpy as np
import cv2

class TrafficDetect:
    frame = None
    
    def __init__(self):
        pass
    
    def setLabel(self,img,pts, label):
        (x,y,w,h) = cv2.boundingRect(pts)
        pt1 = (x,y)
        pt2 = (x+w, y+h)
        cv2.rectangle(img,pt1,pt2,(0,255,0),2)
        cv2.putText(img,label,(pt1[0],pt1[1]-3), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255))
    
    def set_roi_color_traf(self, frame, x_len, start_y, offset_y):
        _, width, _ = frame.shape
        #컬러 전용은 shape에서 리턴3개 나온다.  흑백은 리턴2개 나옴. 
        start_x = int(width/2 - (x_len/2))
        end_x = int(width - start_x)
        return frame[start_y:start_y+offset_y, start_x:end_x], start_x, start_y

    def traf_det(self, image):
        ##불켜진 상태에서 신호등 사각형 찾기
        sign_roi,_,_ = self.set_roi_color_traf(image,200,80,100)
        H,L,S = cv2.split(cv2.cvtColor(sign_roi,cv2.COLOR_BGR2HLS))
        L = 255-L
        _,L = cv2.threshold(L,130,255, cv2.THRESH_BINARY)
        
        #kernel = np.ones((2,2),np.uint8)
        #dila = cv2.dilate(L,kernel)
        #k = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
        #eros = cv2.erode(L,k)
        _,contours,_ = cv2.findContours(L, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        roi2 = L
        hsv = L
        v =L
        cimg =L
        for cont in contours:
            approx = cv2.approxPolyDP(cont, cv2.arcLength(cont,True) *0.02, True)
            vtc = len(approx)
            #print cont[0]
            if vtc ==4:
                
                ##사각형 최소 폭 이상만 잡기
                ## cont[0][0][0]이 x이고 cont[0][0][1]가 y다. 
                #setLabel(sign_roi, cont, 'traf_Box')
                cont_x_min = 300
                cont_x_max = 0
                cont_x_width =0
                cont_y_min =300
                cont_y_max = 0
                cont_y_width = 0
                for j in cont:
                    i = j[0][0]
                    k = j[0][1]
    
                    #print("")
                    if cont_x_max <i:
                        cont_x_max = i
                    if cont_x_min >i:
                        cont_x_min =i
                    if cont_y_max <k:
                        cont_y_max = k
                    if cont_y_min >k:
                        cont_y_min = k
                cont_x_width = cont_x_max - cont_x_min
                cont_y_width = cont_y_max - cont_y_min
                
                #print cont_y_width
                
                if 100 >cont_x_width >40 and 50>cont_y_width >15:
                    #print "1"
                    #print(cont_x_width, cont_y_width)
                    self.setLabel(sign_roi, cont, 'traf_Box')
                    (x,y,w,h) = cv2.boundingRect(cont)  
                    print (x,y,w,h)
                    roi2 = sign_roi[y:y+h,x:x+w].copy()
                    hsv = cv2.cvtColor(roi2, cv2.COLOR_BGR2HSV)  
                    h, s, v = cv2.split(hsv)
                    _,v = cv2.threshold(v, 200,255,cv2.THRESH_BINARY)
                    cimg = cv2.cvtColor(v,cv2.COLOR_GRAY2BGR)  
                    circles = cv2.HoughCircles(v, cv2.HOUGH_GRADIENT, 1,20, param1=180, param2=10, minRadius=3, maxRadius=15)
                    try:
                        circles = np.uint16(np.around(circles))
                        for i in circles[0,:]:
                            cv2.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
                            cv2.circle(cimg,(i[0],i[1]),2,(0,0,255),3)
                            #중심은 빨간색으로 색칠
                            #반지름이 2라서 매우 작으니까 점처럼 보이지만 원이다. 두께는 3
                            cr_img = v[i[1]-10 : i[1]+10, i[0]-10 : i[0]+10]
                            #원의 중심 좌표 -10에서 +10까지하면 한변 길이 20짜리 정사각형 생김
                            #그 원의 v성분 값을 cr_img로 저장함. 
                        
                            img_str = 'x: {0}, y: {1}, mean : {2}'.format(i[0],i[1], cr_img.mean())
                            #좌표값, v성분들의 평균값mean()을  구함
                            print(img_str) 
                            if 13> i[0] >3:
                                print "Red"
                                return False
                            elif 30>i[0] > 15 and i[0] < 85:
                                print "Yellow"
                                return False
                            elif  i[0] >35 :
                                print "Green"
                                return True
                        
                    except AttributeError:
                        pass
    
                    
        
        #return  cimg
        return False