"""
Ball Detection using HSV color segmentation.
Detects orange and light-blue hanging balls for segments 2 and 4.
"""
import cv2
import numpy as np
import math


# HSV thresholds (tuned for the race environment)
ORANGE_BALL_HSV = {
    'lower': np.array([0, 80, 80]),
    'upper': np.array([25, 255, 255]),
}

LIGHT_BLUE_BALL_HSV = {
    'lower': np.array([85, 80, 80]),
    'upper': np.array([130, 255, 255]),
}

WHITE_BALL_HSV = {
    'lower': np.array([0, 0, 180]),
    'upper': np.array([180, 40, 255]),
}


class BallDetector:
    """Detects colored balls in RGB image using HSV thresholding + contour fitting."""

    def __init__(self,
                 orange_hsv=None,
                 light_blue_hsv=None,
                 min_radius_px=10,
                 max_radius_px=150,
                 ball_height_m=0.20,
                 camera_fx=615.0, camera_fy=615.0,
                 camera_cx=320.0, camera_cy=240.0):
        self.orange_hsv = orange_hsv or ORANGE_BALL_HSV
        self.light_blue_hsv = light_blue_hsv or LIGHT_BLUE_BALL_HSV
        self.min_radius = min_radius_px
        self.max_radius = max_radius_px
        self.ball_height = ball_height_m  # height above ground when hanging
        self.fx = camera_fx
        self.fy = camera_fy
        self.cx = camera_cx
        self.cy = camera_cy

    def detect_color_balls(self, bgr_image):
        """Detect balls of specified color type.

        Returns list of dicts: [{'u': cx, 'v': cy, 'radius': r, 'color': 'orange'}, ...]
        """
        bgr_results = self._detect_color_balls_hsv(
            cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV))
        # Some Gazebo camera plugins publish RGB bytes while cv_bridge exposes
        # them as a raw 3-channel image. Try the opposite channel order too.
        rgb_results = self._detect_color_balls_hsv(
            cv2.cvtColor(bgr_image, cv2.COLOR_RGB2HSV))
        return self._merge_detections(bgr_results + rgb_results)

    def _detect_color_balls_hsv(self, hsv):
        results = []

        for color_name, hsv_range in [('orange', self.orange_hsv),
                                      ('light_blue', self.light_blue_hsv)]:
            mask = cv2.inRange(hsv, hsv_range['lower'], hsv_range['upper'])
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,
                                    cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                                    cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)))

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < self.min_radius ** 2 * math.pi:
                    continue
                (cx, cy), radius = cv2.minEnclosingCircle(cnt)
                if radius < self.min_radius or radius > self.max_radius:
                    continue
                circularity = 4 * math.pi * area / (cv2.arcLength(cnt, True) ** 2 + 1e-6)
                if circularity < 0.5:
                    continue
                results.append({
                    'u': float(cx),
                    'v': float(cy),
                    'radius': float(radius),
                    'color': color_name,
                    'area': float(area),
                    'circularity': float(circularity),
                })

        return results

    def _merge_detections(self, detections):
        merged = []
        for det in sorted(detections, key=lambda d: d.get('area', 0), reverse=True):
            duplicate = False
            for existing in merged:
                same_color = existing['color'] == det['color']
                close = (
                    abs(existing['u'] - det['u']) < 25 and
                    abs(existing['v'] - det['v']) < 25
                )
                if same_color and close:
                    duplicate = True
                    break
            if not duplicate:
                merged.append(det)
        return merged

    def detect_all_balls(self, bgr_image):
        """Detect all balls (orange + light-blue)."""
        return self.detect_color_balls(bgr_image)

    def estimate_ball_distance(self, ball):
        """Estimate ball distance (meters) from pixel radius.

        Ball diameter in reality is 20cm (radius=0.10m), hanging at ball_height.
        """
        r_px = ball['radius']
        if r_px <= 0:
            return float('inf')
        dist = (self.fx + self.fy) / 2.0 * self.ball_height / r_px
        return dist

    def estimate_ball_angle(self, ball):
        """Estimate ball angle from image center (radians)."""
        du = ball['u'] - self.cx
        dv = ball['v'] - self.cy
        angle_x = math.atan2(du, self.fx)
        angle_y = math.atan2(dv, self.fy)
        return angle_x, angle_y

    def ball_to_world_estimate(self, ball, robot_x, robot_y, robot_yaw):
        """Estimate ball position in world frame.

        This is a rough estimate based on pixel radius -> distance.
        For accurate positioning, use depth data.
        """
        dist = self.estimate_ball_distance(ball)
        angle_x, _ = self.estimate_ball_angle(ball)

        # Ball hanging height
        ball_z = self.ball_height
        # Rough 2D position (ignoring pixel's v for now)
        bx = dist * math.cos(angle_x)
        by = dist * math.sin(angle_x)
        bz = ball_z - 0.22  # relative to robot body height

        cos_yaw = math.cos(robot_yaw)
        sin_yaw = math.sin(robot_yaw)
        wx = robot_x + bx * cos_yaw - by * sin_yaw
        wy = robot_y + bx * sin_yaw + by * cos_yaw
        return wx, wy

    def draw_balls(self, image, balls):
        """Draw detected balls on image for debugging."""
        for ball in balls:
            cx = int(ball['u'])
            cy = int(ball['v'])
            r = int(ball['radius'])
            color_map = {'orange': (0, 140, 255),
                         'light_blue': (235, 206, 135)}
            color = color_map.get(ball['color'], (0, 255, 0))
            cv2.circle(image, (cx, cy), r, color, 2)
            cv2.putText(image, ball['color'],
                        (cx - 20, cy - r - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            d = self.estimate_ball_distance(ball)
            cv2.putText(image, f"{d:.2f}m",
                        (cx - 20, cy + r + 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        return image


def quick_test(bgr_image):
    """Quick standalone test on an image."""
    det = BallDetector()
    balls = det.detect_all_balls(bgr_image)
    return balls
