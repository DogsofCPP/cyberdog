"""
Bridge and Course Element Detection for Segment 6.
Detects:
  - Yellow course boundary lines
  - Bridge edges
  - End zone / finish circle
  - Ground plane for visual localization
"""
import cv2
import numpy as np
import math


# HSV thresholds for yellow boundary lines (RGB: 255, 255, 0)
YELLOW_HSV = {
    'lower': np.array([15, 80, 80]),
    'upper': np.array([35, 255, 255]),
}

# White marking thresholds
WHITE_HSV = {
    'lower': np.array([0, 0, 180]),
    'upper': np.array([180, 30, 255]),
}


class BridgeDetector:
    """Detects bridge, boundary lines, and end zone for visual localization."""

    def __init__(self,
                 camera_fx=615.0, camera_fy=615.0,
                 camera_cx=320.0, camera_cy=240.0,
                 ground_height_m=0.0):
        self.fx = camera_fx
        self.fy = camera_fy
        self.cx = camera_cx
        self.cy = camera_cy
        self.ground_height = ground_height_m

    # ------------------------------------------------------------------
    # Yellow boundary line detection
    # ------------------------------------------------------------------
    def detect_yellow_lines(self, bgr_image):
        """Detect yellow boundary lines on the course."""
        hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, YELLOW_HSV['lower'], YELLOW_HSV['upper'])
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,
                                cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                                cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)))

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)
        results = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 100:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            aspect = float(max(w, h)) / (min(w, h) + 1e-6)
            if aspect > 2.0:
                results.append({
                    'type': 'yellow_line',
                    'u': float(x + w // 2),
                    'v': float(y + h // 2),
                    'width': float(w),
                    'height': float(h),
                    'area': float(area),
                    'bounding_box': (x, y, w, h),
                    'is_vertical': h > w,
                })
        return results

    # ------------------------------------------------------------------
    # End zone / finish circle detection
    # ------------------------------------------------------------------
    def detect_end_zone(self, bgr_image):
        """Detect the finish/end zone circle marker.
        
        The end zone is a circular marking at the finish position.
        Looks for yellow/white circular or rectangular markers.
        """
        results = []

        hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        yellow_mask = cv2.inRange(hsv, YELLOW_HSV['lower'], YELLOW_HSV['upper'])
        white_mask = cv2.inRange(hsv, WHITE_HSV['lower'], WHITE_HSV['upper'])
        combined_mask = cv2.bitwise_or(yellow_mask, white_mask)

        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE,
                                         cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7)))

        contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 200:
                continue
            (cx, cy), radius = cv2.minEnclosingCircle(cnt)
            circ = 4 * math.pi * area / (cv2.arcLength(cnt, True) ** 2 + 1e-6)
            if circ > 0.5:
                results.append({
                    'type': 'end_zone',
                    'u': float(cx),
                    'v': float(cy),
                    'radius': float(radius),
                    'area': float(area),
                    'circularity': float(circ),
                })

        return results

    # ------------------------------------------------------------------
    # Ground plane detection (for height estimation)
    # ------------------------------------------------------------------
    def detect_ground_plane(self, bgr_image, depth_image=None):
        """Estimate ground plane position in image.
        
        Returns the v-coordinate (row) where ground meets horizon,
        or None if not detected.
        """
        hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)

        gray = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)
        lines = cv2.HoughLinesP(edges, 1, math.pi / 180,
                                threshold=50, minLineLength=50, maxLineGap=10)

        if lines is None:
            return None

        horizontal_lines = []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            angle = abs(math.atan2(y2 - y1, x2 - x1))
            if angle < 0.3 or angle > math.pi - 0.3:
                y_avg = (y1 + y2) // 2
                horizontal_lines.append(y_avg)

        if horizontal_lines:
            return max(horizontal_lines)
        return None

    # ------------------------------------------------------------------
    # Bridge edge detection (narrow platform detection)
    # ------------------------------------------------------------------
    def detect_bridge_edges(self, bgr_image):
        """Detect bridge edges for localization after jumping.
        
        The bridge is a narrow platform - detect its edges.
        """
        hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2GRAY)

        edges = cv2.Canny(gray, 50, 150)
        lines = cv2.HoughLinesP(edges, 1, math.pi / 180,
                                threshold=30, minLineLength=30, maxLineGap=20)

        results = []
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                length = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
                if length > 30:
                    angle = math.atan2(y2 - y1, x2 - x1)
                    results.append({
                        'type': 'edge',
                        'x1': float(x1), 'y1': float(y1),
                        'x2': float(x2), 'y2': float(y2),
                        'length': float(length),
                        'angle': float(angle),
                    })
        return results

    # ------------------------------------------------------------------
    # Visual localization using boundary lines
    # ------------------------------------------------------------------
    def estimate_pose_from_lines(self, yellow_lines, robot_height_m=0.3):
        """Estimate robot pose from detected boundary lines.
        
        In segment 6, the robot faces -Y (yaw=-90deg).
        Yellow lines should appear on the left side of the image.
        
        Returns (x, y, yaw_deg) estimate or None.
        """
        if not yellow_lines:
            return None

        left_lines = [l for l in yellow_lines if l['is_vertical'] and l['u'] < self.cx]
        if not left_lines:
            return None

        avg_u = sum(l['u'] for l in left_lines) / len(left_lines)
        image_width = 640

        offset_from_center = (self.cx - avg_u) / self.fx
        lateral_offset_m = offset_from_center * 1.0

        est_x = 3.0 + lateral_offset_m
        est_y = 13.0
        est_yaw = -90.0

        return (est_x, est_y, est_yaw)

    # ------------------------------------------------------------------
    # Distance estimation from image features
    # ------------------------------------------------------------------
    def estimate_distance_to_object(self, obj, object_real_size_m=0.10):
        """Estimate distance to an object from pixel size.
        
        For football: radius = 0.10m
        """
        if 'radius' in obj:
            r_px = obj['radius']
            if r_px > 0:
                return self.fx * object_real_size_m / r_px
        return None

    def estimate_angle_to_object(self, obj):
        """Estimate angle to object from image center."""
        du = obj.get('u', self.cx) - self.cx
        angle = math.atan2(du, self.fx)
        return angle

    # ------------------------------------------------------------------
    # Unified detection
    # ------------------------------------------------------------------
    def detect_all(self, bgr_image):
        """Run all detectors and return combined list."""
        results = []
        results.extend(self.detect_yellow_lines(bgr_image))
        results.extend(self.detect_end_zone(bgr_image))
        results.extend(self.detect_bridge_edges(bgr_image))
        return results

    # ------------------------------------------------------------------
    # Debug visualization
    # ------------------------------------------------------------------
    def draw_detection(self, image, detections):
        """Draw detected elements on image for debugging."""
        for det in detections:
            det_type = det.get('type', 'unknown')
            color_map = {
                'yellow_line': (0, 255, 255),
                'end_zone': (255, 255, 0),
                'edge': (200, 200, 100),
            }
            color = color_map.get(det_type, (0, 255, 0))

            if det_type == 'yellow_line':
                x, y, w, h = det['bounding_box']
                cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)
            elif det_type == 'end_zone' and 'radius' in det:
                cv2.circle(image, (int(det['u']), int(det['v'])),
                          int(det['radius']), color, 2)
            elif det_type == 'edge':
                cv2.line(image, (int(det['x1']), int(det['y1'])),
                        (int(det['x2']), int(det['y2'])), color, 2)

            u, v = det.get('u', 0), det.get('v', 0)
            cv2.putText(image, det_type, (int(u) - 30, int(v) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        return image


def quick_test(bgr_image):
    """Quick standalone test on an image."""
    det = BridgeDetector()
    return det.detect_all(bgr_image)
