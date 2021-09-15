#!/usr/bin/env python
# -*-coding:utf-8-*-

import time
import numpy as np
import os
import cv2

class PassengerDetect:

    def __init__(self):


        self.MAN = "man"
        self.CAT = "cat"
        self.last_time = time.time()

        self.image_candidates = {}

        # ORB detector
        # https://bkshin.tistory.com/entry/OpenCV-27-%ED%8A%B9%EC%A7%95-%EB%94%94%EC%8A%A4%ED%81%AC%EB%A6%BD%ED%84%B0-%EA%B2%80%EC%B6%9C%EA%B8%B0-SIFT-SURF-ORB
        #  self.detector = cv2.ORB_create(nfeatures=2000, patchSize=31)
        self.detector = cv2.ORB_create()
        # detector = cv2.ORB_create(nfeatures, scaleFactor, nlevels, edgeThreshold, firstLevel, WTA_K, scoreType, patchSize, fastThreshold)


        # FLANN matcher
        # https://bkshin.tistory.com/entry/OpenCV-28-%ED%8A%B9%EC%A7%95-%EB%A7%A4%EC%B9%ADFeature-Matching
        FLANN_INDEX_LSH = 6
        index_params = dict(algorithm=FLANN_INDEX_LSH, table_number=6, key_size=12, multi_probe_level=1)
        search_params = dict(checks=32)
        self.matcher = cv2.FlannBasedMatcher(index_params, search_params)


        # BFMatcher
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)


        file_path = os.path.dirname(os.path.abspath(__file__))
        img_dir = os.path.join(file_path, './images')
        for img_file_name in os.listdir(img_dir):
            img_name = '.'.join(img_file_name.split('.')[0:-1])
            img_path = os.path.join(img_dir, img_file_name)
            img = cv2.imread(img_path)
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            keypoints, descriptor = self.detector.detectAndCompute(img, None)
            self.image_candidates[img_name] = (img, keypoints, descriptor)

    def find_man(self, cam_image):
        # 감마
        g = 1.1
        out = cam_image.copy()
        out = out.astype(np.float)
        out = ((out / 255) ** (1 / g)) * 255
        out = out.astype(np.uint8)
        cam_image = out

        cam_image = cv2.cvtColor(cam_image, cv2.COLOR_BGR2GRAY)

        cam_kp, cam_desc = self.detector.detectAndCompute(cam_image, None)
        man_img, man_kp, man_desc = self.image_candidates[self.MAN]
        #matches, matchesMask = self.compare_image(cam_desc, man_desc)
        matches, good_matches = self.compare_image(cam_desc, man_desc)

        now = time.time()
        delta_time = now - self.last_time
        if delta_time > 0.2:

#            draw_params = dict(matchColor = (-1,-1,-1),
#                   singlePointColor = (255,0,0),
#                   #matchesMask = matchesMask,
#                   flags = cv2.DrawMatchesFlags_DEFAULT)
#
#            img3 = cv2.drawMatchesKnn(cam_image, cam_kp, man_img, man_kp, matches, None, **draw_params)
            img3 = cv2.drawMatches(cam_image, cam_kp, man_img, man_kp, good_matches, None, flags=cv2.DRAW_MATCHES_FLAGS_NOT_DRAW_SINGLE_POINTS)
            path = "/home/nvidia/Grepp-Xytron_Final_Project/src/final2/src/detect"
            cv2.imwrite(path + "/images/{}.jpg".format(now), img3)
            print('image saved')
            self.last_time = now

        # return similarity
        return 

    def find_cat(self, cam_image):
        cam_kp, cam_desc = self.detector.detectAndCompute(cam_image, None)
        _, _, man_desc = self.image_candidates[self.CAT]
        similarity = self.compare_image(cam_desc, man_desc)

        return similarity

    def compare_image(self, desc_1, desc_2):
        # Match descriptors.
        matches = self.bf.match(desc_1, desc_2)

        # Sort them in the order of their distance.
        matches = sorted(matches, key = lambda x: x.distance)

        
        min_dist, max_dist = matches[0].distance, matches[-1].distance
        ratio = 0.1
        good_thresh = (max_dist - min_dist) * ratio + min_dist
        good_matches = [m for m in matches if m.distance < good_thresh]

        print("matches: {}/{}({}), min: {}, max: {}, thresh: {}".format(len(good_matches), len(matches), float(len(good_matches))/len(matches),  min_dist, max_dist, good_thresh))

        return matches, good_matches




#        matches = self.matcher.knnMatch(desc_1, desc_2, k=2)
#
#        # Need to draw only good matches, so create a mask
#        # matchesMask = [[0,0] for i in range(len(matches))]
#
#        ratio = 0.75
#        good_matches = []
#        for i, m_n in enumerate(matches):
#            if len(m_n) != 2:
#                continue
#            (m, n) = m_n
#            if m.distance < ratio * n.distance:
#                good_matches
#
#        print("matches: {}/{}({})".format(len(good_matches), len(matches), float(len(good_matches))/len(matches)))
#
#        return matches, good_matches

        # good_matches = []
        for i, m_n in enumerate(matches):
            if len(m_n) != 2:
                continue
            (m, n) = m_n
            if m.distance < ratio * n.distance:
                matchesMask[i] = [1, 0]

        # similarity = len(good_matches)



        return matches, matchesMask

        # good_matches = [first for first,second in matches if first.distance < second.distance * ratio]
        # mats[name_1] = len(good_matches)

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

        # return mats
