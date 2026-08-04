"""
Yellow Boundary Line Detection using color segmentation.
Detects the yellow race boundary lines for line-following in segments 1 and 3.
"""
import cv2
import numpy as np


# HSV thresholds for yellow
YELLOW_HSV = {
    'lower': np.array([15, 80, 80]),
    'upper': np.array([40, 255, 255]),
}


class BoundaryDetector:
    """Detects yellow boundary lines for line-following navigation."""

    def __init__(self,
                 yellow_hsv=None,
                 camera_cx=320.0,
                 camera_cy=240.0,
                 camera_fx=615.0):
        self.yellow_hsv = yellow_hsv or YELLOW_HSV
        self.cx = camera_cx
        self.cy = camera_cy
        self.fx = camera_fx

    def detect_yellow_lines(self, bgr_image):
        """Detect yellow lines and return their image-space centroid.

        Returns (center_u, center_v, width_px) or (None, None, None) if not found.
        """
        hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.yellow_hsv['lower'], self.yellow_hsv['upper'])

        # Clean up mask
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None, None, None

        # Find the largest yellow contour (most likely the boundary line)
        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        if area < 50:
            return None, None, None

        M = cv2.moments(largest)
        if M['m00'] <= 0:
            return None, None, None

        cx = M['m10'] / M['m00']
        cy = M['m01'] / M['m00']
        x, y, w, h = cv2.boundingRect(largest)

        return cx, cy, w

    def detect_left_right_lines(self, bgr_image):
        """Detect left and right yellow boundaries separately.

        Returns (left_u, right_u) or (None, None).
        Splits image into left/right halves.
        """
        h, w = bgr_image.shape[:2]
        hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.yellow_hsv['lower'], self.yellow_hsv['upper'])

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        left_mask = mask[:, :w // 2]
        right_mask = mask[:, w // 2:]

        left_cx, right_cx = None, None

        lcnt, _ = cv2.findContours(left_mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
        if lcnt:
            largest = max(lcnt, key=cv2.contourArea)
            M = cv2.moments(largest)
            if M['m00'] > 0:
                left_cx = M['m10'] / M['m00']  # relative to left half

        rcnt, _ = cv2.findContours(right_mask, cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
        if rcnt:
            largest = max(rcnt, key=cv2.contourArea)
            M = cv2.moments(largest)
            if M['m00'] > 0:
                right_cx = M['m10'] / M['m00'] + w // 2  # absolute x

        return left_cx, right_cx

    def compute_lateral_error(self, bgr_image, target_v_px=None):
        """Compute lateral (horizontal) error of robot from center of lane.

        Returns:
          lateral_error: positive = robot is to the right of lane center
          confidence: how confident the detection is (0..1)
        """
        h, w = bgr_image.shape[:2]
        target_v = target_v_px or (h * 0.7)  # look at lower part of image

        hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.yellow_hsv['lower'], self.yellow_hsv['upper'])

        # Focus on lower half where the line should be closest
        bottom_mask = mask[int(h * 0.5):, :]
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        bottom_mask = cv2.morphologyEx(bottom_mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(bottom_mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return 0.0, 0.0

        all_points = np.vstack([c for c in contours])
        M = cv2.moments(all_points)
        if M['m00'] <= 0:
            return 0.0, 0.0

        cy = M['m01'] / M['m00']
        cx = M['m10'] / M['m00']

        # Compute lane center (approximate)
        # For a single yellow line on the right side:
        # positive error = robot is left of the line
        image_center = w / 2.0
        lateral_error_px = cx - image_center

        # Convert to approximate angle
        angle_error = np.arctan2(lateral_error_px, self.fx)
        confidence = min(1.0, cv2.contourArea(max(contours, key=cv2.contourArea)) / 500)

        return float(angle_error), float(confidence)

    def draw_boundary(self, image, cx=None, cy=None, w_px=None):
        """Draw detected boundary for debugging."""
        if cx is not None and cy is not None:
            cv2.circle(image, (int(cx), int(cy)), 8, (0, 255, 255), -1)
        if w_px is not None:
            cy_int = int(cy) if cy else image.shape[0] // 2
            cv2.line(image,
                     (int(cx) - w_px // 2, cy_int),
                     (int(cx) + w_px // 2, cy_int),
                     (0, 255, 255), 2)
        return image
