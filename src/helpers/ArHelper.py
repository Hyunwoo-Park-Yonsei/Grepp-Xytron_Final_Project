#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
from tf.transformations import euler_from_quaternion
import math


class ArHelper:
    def __init__(self):
        pass
    
    def ArData(self,data):
        arData = {}
        
        for i in data:
            arNum = i. id
            
            arData["DX"] = i.pose.pose.position.x
            arData["DY"] = i.pose.pose.position.y
            arData["DZ"] = i.pose.pose.position.z
            
            arData["AX"] = i.pose.pose.orientation.x
            arData["AY"] = i.pose.pose.orientation.y
            arData["AZ"] = i.pose.pose.orientation.z
            arData["AW"] = i.pose.pose.orientation.w
            _,pitch,_ = euler_from_quaternion((arData["AX"],arData["AY"],arData["AZ"], arData["AW"]))
            pitch = math.degrees(pitch) * -1
            dist = math.sqrt(pow(arData["DX"], 2) + pow(arData["DZ"], 2))
            
            
            return arNum, dist