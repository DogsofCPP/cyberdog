"""
Object Detection for segment 4 targets:
  - Coke bottle (large cylinder, black/dark)
  - Football / soccer ball (white sphere)
  - Height-limit bar (red bar at 40cm height)
  - Square obstacle (20x20x20cm, light blue in simulation)
"""
import cv2
import numpy as np
import math


# HSV thresholds for various objects
RED_HSV = {
    'lower1': np.array([0, 80, 80]),
    'upper1': np.array([10, 255, 255]),
    'lower2': np.array([160, 80, 80]),
    'upper2': np.array([180, 255, 255]),
}

# 青灰色限高杆 (仿真/真实环境)
CYAN_GRAY_HSV = {
    'lower': np.array([85, 15, 70]),    # 青色/灰青色
    'upper': np.array([145, 70, 130]),
}

BLACK_HSV = {
    'lower': np.array([0, 0, 0]),
    'upper': np.array([180, 80, 80]),
}

WHITE_HSV = {
    'lower': np.array([0, 0, 200]),
    'upper': np.array([180, 30, 255]),
}

LIGHT_BLUE_HSV = {
    'lower': np.array([85, 60, 60]),
    'upper': np.array([130, 255, 255]),
}


class ObjectDetector:
    """Detects segment-4 target objects using color and shape cues."""

    def __init__(self,
                 camera_fx=615.0, camera_fy=615.0,
                 camera_cx=320.0, camera_cy=240.0):
        self.fx = camera_fx
        self.fy = camera_fy
        self.cx = camera_cx
        self.cy = camera_cy

    # ------------------------------------------------------------------
    # Coke bottle (dark cylinder)
    # ------------------------------------------------------------------
    def detect_coke(self, bgr_image):
        """Detect coke bottle (dark cylinder on ground)."""
        hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, BLACK_HSV['lower'], BLACK_HSV['upper'])
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,
                                cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7)))

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        results = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 100:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            aspect = float(h) / (w + 1e-6)
            # Cylinder: taller than wide
            if aspect > 1.5 and h > 20:
                results.append({
                    'type': 'coke',
                    'u': float(x + w // 2),
                    'v': float(y + h // 2),
                    'width': float(w),
                    'height': float(h),
                    'area': float(area),
                    'bounding_box': (x, y, w, h),
                })
        return results

    # ------------------------------------------------------------------
    # Football (white sphere)
    # ------------------------------------------------------------------
    def detect_football(self, bgr_image):
        """Detect white football / soccer ball."""
        hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, WHITE_HSV['lower'], WHITE_HSV['upper'])
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,
                                cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        results = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 50:
                continue
            (cx, cy), radius = cv2.minEnclosingCircle(cnt)
            circ = 4 * math.pi * area / (cv2.arcLength(cnt, True) ** 2 + 1e-6)
            if circ > 0.5:
                results.append({
                    'type': 'football',
                    'u': float(cx),
                    'v': float(cy),
                    'radius': float(radius),
                    'area': float(area),
                })
        return results

    # ------------------------------------------------------------------
    # Height-limit bar (cyan-gray horizontal bar at ~40cm height)
    # ------------------------------------------------------------------
    def detect_height_bar(self, bgr_image):
        """Detect cyan-gray horizontal bar at ~40cm from ground.

        The bar is cyan-gray, long horizontally, thin vertically.
        In the image it will appear as a horizontal line at upper portion.
        """
        hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        # 青灰色检测 (仿真环境)
        mask = cv2.inRange(hsv, CYAN_GRAY_HSV['lower'], CYAN_GRAY_HSV['upper'])

        # Close small gaps
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 3))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        results = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 50:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            aspect = float(w) / (h + 1e-6)
            # Bar: wider than tall
            if aspect > 5.0:
                results.append({
                    'type': 'height_bar',
                    'u': float(x + w // 2),
                    'v': float(y + h // 2),
                    'width': float(w),
                    'height': float(h),
                    'area': float(area),
                    'bounding_box': (x, y, w, h),
                })
        return results

    # ------------------------------------------------------------------
    # Square obstacle (light blue 20x20x20cm blocks)
    # ------------------------------------------------------------------
    def detect_square_obstacle(self, bgr_image):
        """Detect square obstacles (light blue blocks)."""
        hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, LIGHT_BLUE_HSV['lower'], LIGHT_BLUE_HSV['upper'])
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,
                                cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7)))

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        results = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 100:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            approx = cv2.approxPolyDP(cnt, 0.05 * cv2.arcLength(cnt, True), True)
            results.append({
                'type': 'obstacle',
                'u': float(x + w // 2),
                'v': float(y + h // 2),
                'width': float(w),
                'height': float(h),
                'area': float(area),
                'corners': len(approx),
                'bounding_box': (x, y, w, h),
            })
        return results

    # ------------------------------------------------------------------
    # Unified detection
    # ------------------------------------------------------------------
    def detect_all(self, bgr_image):
        """Run all detectors and return combined list."""
        results = []
        results.extend(self.detect_coke(bgr_image))
        results.extend(self.detect_football(bgr_image))
        results.extend(self.detect_height_bar(bgr_image))
        results.extend(self.detect_square_obstacle(bgr_image))
        return results

    def draw_objects(self, image, objects):
        """Draw detected objects on image."""
        color_map = {
            'coke': (64, 64, 64),
            'football': (255, 255, 255),
            'height_bar': (105, 105, 94),  # 青灰色
            'obstacle': (235, 206, 135),
        }
        for obj in objects:
            color = color_map.get(obj['type'], (0, 255, 0))
            cv2.putText(image, obj['type'],
                        (int(obj['u']) - 30, int(obj['v']) - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            cv2.circle(image, (int(obj['u']), int(obj['v'])), 5, color, -1)
        return image
