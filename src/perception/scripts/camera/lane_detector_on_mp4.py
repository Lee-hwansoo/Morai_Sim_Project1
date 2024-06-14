#!/usr/bin/env python3
# -*- coding: utf -8 -*-

import cv2
import numpy as np
from sklearn import linear_model
import random
import math

from warnings import simplefilter
from sklearn.exceptions import ConvergenceWarning
simplefilter("ignore", category=ConvergenceWarning)

class IMGParser:
    def __init__(self, video_file='output.mp4'):
        self.video_file = video_file
        self.cap = cv2.VideoCapture(self.video_file)
        self.is_image = False
        self.img_bgr = None
        self.img_warp = None
        self.img_lane_edge = None
        self.img_lane_fit = None

        # 0.225, 0.425
        self.warp_points = np.array([
            [144, 370],
            [272, 260],
            [368, 260],
            [496, 370]
        ])

    def read_frame(self):
        ret, frame = self.cap.read()
        if ret:
            self.img_bgr = frame
            self.is_image = True
        else:
            self.is_image = False

    def preprocess(self, image):
        image = self.bgr_to_hls(image)
        image = self.grayscale(image)
        image = self.gaussian_blur(image)
        self.img_lane_edge = self.canny_edge_detection(image)
        return self.img_lane_edge

    def warp_image(self, image):
        height, width = image.shape[:2]

        destination_points = np.float32([
            [0, height],
            [0, 0],
            [width, 0],
            [width, height]
        ])

        perspective_transform = cv2.getPerspectiveTransform(np.float32(self.warp_points), destination_points)

        self.img_warp = cv2.warpPerspective(image, perspective_transform, (width, height), flags=cv2.INTER_LINEAR)

        return self.img_warp

    def bgr_to_hls(self, image):
        hls_image = cv2.cvtColor(image, cv2.COLOR_BGR2HLS)

        lower_yellow = np.array([20, 120, 80], dtype=np.uint8)
        upper_yellow = np.array([45, 200, 255], dtype=np.uint8)
        yellow_mask = cv2.inRange(hls_image, lower_yellow, upper_yellow)

        lower_white = np.array([0, 200, 0], dtype=np.uint8)
        upper_white = np.array([255, 255, 255], dtype=np.uint8)
        white_mask = cv2.inRange(hls_image, lower_white, upper_white)

        mask = cv2.bitwise_or(yellow_mask, white_mask)

        height, width, _ = image.shape
        x_min = 150
        x_max = 490
        x_mask = np.ones((height, width), dtype=np.uint8) * 255
        x_mask[:, x_min:x_max] = 0

        mask = cv2.bitwise_and(mask, x_mask)

        filtered = cv2.bitwise_and(image, image, mask=mask)

        return filtered

    def grayscale(self, image):
        return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    def gaussian_blur(self, image, kernel_size=5):
        return cv2.GaussianBlur(image, (kernel_size, kernel_size), 0)

    def canny_edge_detection(self, image, low_threshold=60, high_threshold=70):
        return cv2.Canny(image, low_threshold, high_threshold)

    def recon_lane_pts(self, img):
        if cv2.countNonZero(img) != 0:
            lane_pts = cv2.findNonZero(img).reshape([-1,2])
            return lane_pts
        else:
            return None

    def draw_lane_img(self, img, left_lines, right_lines, color=(0, 255, 0)):
        if left_lines is not None and right_lines is not None:
            for line in left_lines:
                x1, y1, x2, y2 = line[0]
                img = cv2.line(img, (x1, y1), (x2, y2), color, thickness=10)

            for line in right_lines:
                x1, y1, x2, y2 = line[0]
                img = cv2.line(img, (x1, y1), (x2, y2), color, thickness=10)

        self.img_lane_fit = img
        return self.img_lane_fit

class HoughLaneDetector:
    def __init__(self):
        pass

    def detect_lanes(self, image):
        lines = cv2.HoughLinesP(image, rho=1, theta=np.pi/180, threshold=50, minLineLength=65, maxLineGap=500)
        return lines

    def separate_lanes(self, lines, img_width):
        left_lines = []
        right_lines = []

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                slope = (y2 - y1) / (x2 - x1)

                # 수평선 제외
                if abs(slope) < 0.5:
                    continue

                if slope < 0 and x1 < img_width / 2 and x2 < img_width / 2:  # Left lane
                    left_lines.append(line)
                elif slope > 0 and x1 > img_width / 2 and x2 > img_width / 2:  # Right lane
                    right_lines.append(line)

        return left_lines, right_lines

    def extract_weighted_average_lane(self, lines):
        if len(lines) == 0:
            return None

        total_length = 0
        weighted_sum_x1 = 0
        weighted_sum_y1 = 0
        weighted_sum_x2 = 0
        weighted_sum_y2 = 0

        # Calculate total length of all lines and weighted sum of coordinates
        for line in lines:
            x1, y1, x2, y2 = line[0]
            length = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
            total_length += length
            weighted_sum_x1 += length * x1
            weighted_sum_y1 += length * y1
            weighted_sum_x2 += length * x2
            weighted_sum_y2 += length * y2

        # Calculate weighted averages
        mean_x1 = int(weighted_sum_x1 / total_length)
        mean_y1 = int(weighted_sum_y1 / total_length)
        mean_x2 = int(weighted_sum_x2 / total_length)
        mean_y2 = int(weighted_sum_y2 / total_length)

        # Return the weighted average lane
        return [[[mean_x1, mean_y1, mean_x2, mean_y2]]]

if __name__ == '__main__':
    image_parser = IMGParser('output_video.mp4')
    lane_detector = HoughLaneDetector()

    while True:
        image_parser.read_frame()

        if image_parser.is_image:
            img_warp = image_parser.warp_image(image_parser.img_bgr)
            img_lane_edge = image_parser.preprocess(img_warp)
            lines = lane_detector.detect_lanes(img_lane_edge)
            left_lines, right_lines = lane_detector.separate_lanes(lines, img_warp.shape[1])

            if len(left_lines) != 0 and len(right_lines) != 0:
                left_average_lane = lane_detector.extract_weighted_average_lane(left_lines)
                right_average_lane = lane_detector.extract_weighted_average_lane(right_lines)
                if left_average_lane is not None and right_average_lane is not None:
                    img_warp = image_parser.draw_lane_img(np.copy(img_warp), left_average_lane, right_average_lane, color=(0, 0, 255))

            concatenated_image = np.concatenate((image_parser.img_bgr, img_warp), axis=1)
            cv2.imshow("image", concatenated_image)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            break

    image_parser.cap.release()
    cv2.destroyAllWindows()
