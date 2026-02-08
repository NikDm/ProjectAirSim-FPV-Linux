"""
Copyright (C) Microsoft Corporation. 
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.

Laser tracking class for detecting and tracking laser points in camera frames.
"""

import cv2
import numpy as np
import imutils
from collections import deque
import time
import math

from projectairsim.utils import projectairsim_log


class LaserTracker:
    """Laser tracking class that processes camera frames for laser point detection."""
    
    def __init__(self, buffer_size=64, min_radius=1):
        # Define the lower and upper boundaries of the "green" laser point in HSV color space
        # Green detection ranges - adjusted for better small laser point detection
        self.green_lower = (35, 40, 40)   # Green hue range: 35-85, lower saturation/value for sensitivity
        self.green_upper = (85, 255, 255)   # Green saturation and value ranges
        
        self.pts = deque(maxlen=buffer_size)
        
        # Minimum radius for laser point detection (lowered for small laser point detection)
        self.min_radius = min_radius
        
        # FPS calculation variables
        self.fps_counter = 0
        self.fps_start_time = time.time()
        self.fps = 0
        
        # Laser tracking state
        self.laser_center = None
        self.frame_center = None
        self.laser_detected = False
        
    def process_frame(self, frame):
        """Process a single frame for laser tracking and return annotated frame."""
        if frame is None:
            return None, None, None
            
        # Calculate FPS
        self.fps_counter += 1
        if self.fps_counter >= 30:  # update FPS every 30 frames
            self.fps = self.fps_counter / (time.time() - self.fps_start_time)
            self.fps_counter = 0
            self.fps_start_time = time.time()
        
        # Resize the frame to preserve detail for small laser point detection
        # Using larger width to maintain resolution for small objects
        frame = imutils.resize(frame, width=800)
        # Reduced blur kernel size to preserve small laser point details
        blurred = cv2.GaussianBlur(frame, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        
        # Store frame center for laser position calculation
        self.frame_center = (frame.shape[1] // 2, frame.shape[0] // 2)
        
        # Construct a mask for the green color
        # Reduced erosion/dilation to preserve small laser point detections
        mask = cv2.inRange(hsv, self.green_lower, self.green_upper)
        # Minimal morphological operations to avoid removing small laser points
        mask = cv2.erode(mask, None, iterations=1)
        mask = cv2.dilate(mask, None, iterations=1)
        
        # Find contours in the mask and initialize the current
        # (x, y) center of the laser point
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None
        radius = 0
        
        # Only proceed if at least one contour was found
        if len(cnts) > 0:
            # Find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            if M["m00"] != 0:  # Avoid division by zero
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            else:
                center = (int(x), int(y))
            
            # Only proceed if the radius meets a minimum size (lowered for small laser point detection)
            if radius > self.min_radius:
                # Update laser tracking state
                self.laser_center = center
                self.laser_detected = True
                
                # Draw the circle and centroid on the frame
                cv2.circle(frame, (int(x), int(y)), int(radius),
                    (0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)
                
                # Draw line from frame center to laser center
                cv2.line(frame, self.frame_center, center, (255, 0, 0), 2)
                
                # Log the laser coordinates to console
                # print(f"Green laser point detected - x={center[0]}, y={center[1]}, radius={int(radius)}, FPS: {self.fps:.1f}")
            else:
                self.laser_detected = False
        else:
            self.laser_detected = False
        
        # Update the points queue
        self.pts.appendleft(center)
        
        # Loop over the set of tracked points
        for i in range(1, len(self.pts)):
            # If either of the tracked points are None, ignore them
            if self.pts[i - 1] is None or self.pts[i] is None:
                continue
            
            # Otherwise, compute the thickness of the line and
            # draw the connecting lines
            thickness = int(np.sqrt(len(self.pts) / float(i + 1)) * 1)
            cv2.line(frame, self.pts[i - 1], self.pts[i], (0, 0, 255), thickness)
        
        # Draw FPS and laser status on the frame
        cv2.putText(frame, f"FPS: {self.fps:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        status_text = "LASER DETECTED" if self.laser_detected else "NO LASER"
        cv2.putText(frame, status_text, (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0) if self.laser_detected else (0, 0, 255), 2)
        
        return frame, center, mask
    
    def get_laser_direction(self):
        """Calculate the direction from frame center to laser center."""
        if not self.laser_detected or self.laser_center is None or self.frame_center is None:
            return None
        
        # Calculate offset from frame center
        dx = self.laser_center[0] - self.frame_center[0]
        dy = self.laser_center[1] - self.frame_center[1]

        projectairsim_log().info(f"Laser direction - dx: {dx}, dy: {dy}")
        # Calculate angle (in radians) from frame center to laser
        angle = math.atan2(dx, dy)  # Using dy as reference for forward direction
        
        return angle

