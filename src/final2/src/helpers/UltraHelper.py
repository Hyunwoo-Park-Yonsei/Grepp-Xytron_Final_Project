#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import cv2

class UltraHelper:
    def __init__(self):
        pass

    def ultra_get(self, size, ultra_data):
        pixels = (size[1], size[0], 3)
        zeros = np.zeros(pixels, np.uint8)

        # 초음파센서 데이터 출력
        #print("ultra_data:", ultra_data)
        side_right = ultra_data[0]
        side_left = ultra_data[4]
        rear_right = ultra_data[7]
        rear_center = ultra_data[6]
        rear_left = ultra_data[5]

        #time.sleep(0.5)
        # 천천히 출력
        # 다음에 그래픽 만들기
        scale = 6
        scale_sin60 = int(scale * 0.5)
        scale_cos60 = int(scale * 0.8)

        ##640의 절반인 320이 카메라 가운데가 아니라 300
        zeros = cv2.circle(zeros, (300-side_left*scale,10), 6, (100,255,200), 20)
        zeros = cv2.circle(zeros, (300+side_right*scale,10), 6, (100,255,200), 20)
        zeros = cv2.circle(zeros, (300+rear_right*scale_cos60,rear_right*scale_sin60), 6, (100,255,200), 20)
        zeros = cv2.circle(zeros, (300-rear_left*scale_cos60,rear_left*scale_sin60), 6, (100,255,200), 20)
        zeros = cv2.circle(zeros, (300,rear_center*scale), 6, (100,255,200), 20)
        zeros = cv2.circle(zeros, (300,-50), 500, (0,0,255), 5)

        #한계선
        return zeros
