#!/usr/bin/env python
# -*-coding:utf-8-*-

import numpy as np
import os
import cv2

class ImageDetect:

    def __init__(self, frame):
        self.frame = frame
        self.imageCandidates = []


        # ORB detector
        # https://bkshin.tistory.com/entry/OpenCV-27-%ED%8A%B9%EC%A7%95-%EB%94%94%EC%8A%A4%ED%81%AC%EB%A6%BD%ED%84%B0-%EA%B2%80%EC%B6%9C%EA%B8%B0-SIFT-SURF-ORB
        self.detector = cv2.ORB_create()
        # detector = cv2.ORB_create(nfeatures, scaleFactor, nlevels, edgeThreshold, firstLevel, WTA_K, scoreType, patchSize, fastThreshold)


        # FLANN matcher
        # https://bkshin.tistory.com/entry/OpenCV-28-%ED%8A%B9%EC%A7%95-%EB%A7%A4%EC%B9%ADFeature-Matching
        FLANN_INDEX_LSH = 6
        index_params = dict(algorithm=FLANN_INDEX_LSH, table_number=6, key_size=12, multi_probe_level=1)
        search_params = dict(checks=32)
        self.matcher = cv2.FlannBasedMatcher(index_params, search_params)


        dir = os.path.dirname(os.path.abspath(__file__))
        img_dir = os.path.join(file_path, './images')
        for img_file_name in os.listdir(img_dir):
            img_name = '.'.join(img_file_name.split('.')[0:-1])
            img_path = os.path.join(img_dir, img_file_name)
            img = cv2.imread(img_path)
            keypoints, descriptor = self.detector.detectAndCompute(img, None)
            self.imageCandidates.append((img_name, img, keypoints, descriptor))


    def detectImage(self, x_len, start_y, offset_y):
        img = cv2.imread('../images/yellow_cow.jpeg')
        # gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        mats = {}

        for name_1, img_1, kp_1, desc_1 in self.imageCandidates:
            matches = self.matcher.knnMatch(des1,des2,k=2)
            mats[name_1] = len(matches)

        # # Need to draw only good matches, so create a mask
        # matchesMask = [[0,0] for i in range(len(matches))]
        # # ratio test as per Lowe's paper
        # for i,(m,n) in enumerate(matches):
        #     if m.distance < 0.7*n.distance:
        #         matchesMask[i]=[1,0]

        # draw_params = dict(matchColor = (0,255,0),
        #                    singlePointColor = (255,0,0),
        #                    matchesMask = matchesMask,
        #                    flags = cv.DrawMatchesFlags_DEFAULT)

        return mats
