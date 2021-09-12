#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2

class ImageHelper:
    def __init__(self):
        pass

    def img_processing(self, image_raw):
        image_undistorted = self.calibrate_image(image_raw)
        image_gaussian_bluured = cv2.GaussianBlur(image_undistorted, (9,9), 0)

        image_edge = cv2.Canny(np.uint8(blur), self.CANNY_THRESHOLD_LOW, self.CANNY_THRESHOLD_HIGH)

        kernel = np.ones((4,4), np.uint8)
        image_dilated = cv2.dilate(image_edge, kernel)

        return image_dilated, image_undistorted

    def calibrate_image(self, frame):
        """
        이미지 한장 읽을 때마다 위에서 구한 보정 행렬값을 적용하여 이미지를 반듯하게 수정하는 함수
        """

        #보정행렬값을 적용하여 반듯하게 수정하는 함수
        tf_image = cv2.undistort(frame, self.CAMERA_MATRIX, self.DISTORTION_COEFFS, None, self.OPTIMAL_CAMERA_MATRIX)
        x, y, w, h = self.OPTIMAL_CAMERA_ROI
        tf_image = tf_image[y:y+h, x:x+w]

        ##변환 전과 후의 4개 점 좌표를 전달해서 이미지를 원근 변환처리한다.
        return cv2.resize(tf_image, (self.IMAGE_WIDTH, self.IMAGE_HEIGHT))

    def warp_image(self, img):
        #pts1 =np.float32([[228,290],[75,385],[423,290],[573,385]])
        #pts2 =np.float32([[228,290],[228,480],[423,290],[423,480]])

        pts1 = np.float32([[216,245],[34,358],[380,245],[535,358]])
        pts2 = np.float32([[216,245],[216,480],[380,245],[380,480]])

        M = cv2.getPerspectiveTransform(pts1, pts2)
        Minv = cv2.getPerspectiveTransform(pts2, pts1)

        size = (640, 480)
        dst = cv2.warpPerspective(img, M, size, flags=cv2.INTER_LINEAR)
        _, dst2 = cv2.threshold(dst, lane_bin_th, 255, cv2.THRESH_BINARY)

        ##자이카 카메라로 촬영한 동영상이므로 전용 보정값 써야햔다.
        return Minv, dst2

    def black2white(self, image):
        """
        검게 보이는 부분 하얗게 날리기
        """

        width = 640
        height = 480

        # 좌삼각
        point = np.array([[0, 235], [0, height], [204, height]], np.int32)
        image = cv2.fillConvexPoly(image, point, 255)
        # 우삼각
        point = np.array([[415, height], [width, height], [width, 235]], np.int32)
        image = cv2.fillConvexPoly(image, point, 255)

        # return image

        # crop
        crop_img = image[200:450, 120:480]
        return crop_img
