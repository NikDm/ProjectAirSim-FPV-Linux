"""
Copyright (C) Microsoft Corporation. 
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.

Laser steering controller for automatically keeping laser point centered and flying towards it.
"""

import time
from projectairsim.utils import projectairsim_log


class LaserSteeringController:
    """Controller for automatically steering the drone to keep laser point centered."""
    
    def __init__(self, ardu_controller, laser_tracker):
        self.ardu_controller = ardu_controller
        self.laser_tracker = laser_tracker
        self.is_active = False
        self.last_laser_time = 0
        self.laser_lost_timeout = 2.0  # seconds to wait before stopping if laser is lost
        
        # PID-like control parameters for smooth movement
        # These control the MANUAL_CONTROL inputs (-1000 to +1000)
        self.kp_pitch = 0.3    # Proportional gain for pitch (forward/backward)
        self.kp_roll = 0.3      # Proportional gain for roll (left/right)
        self.kp_yaw = 0.2       # Proportional gain for yaw (rotation)
        self.kp_throttle = 0.1  # Proportional gain for throttle adjustment
        
        # Base throttle (hover point)
        self.base_throttle = 500  # 0-1000 range, 500 = hover
        
        # Control thresholds (pixels)
        self.x_threshold = 20  # pixels - threshold for lateral correction
        self.y_threshold = 20  # pixels - threshold for vertical correction
        
        # Maximum control inputs
        self.max_pitch = 300   # Maximum pitch input (-1000 to +1000)
        self.max_roll = 300    # Maximum roll input
        self.max_yaw = 200     # Maximum yaw input
        self.max_throttle_delta = 100  # Maximum throttle adjustment from base
        
    def start(self):
        """Start laser steering mode."""
        self.is_active = True
        projectairsim_log().info("Laser steering mode activated")
    
    def stop(self):
        """Stop laser steering mode."""
        self.is_active = False
        projectairsim_log().info("Laser steering mode deactivated")
    
    def update(self, manual_roll=0, manual_pitch=0, manual_throttle=500, manual_yaw=0):
        """
        Update control based on laser position.
        
        Args:
            manual_roll: Manual roll input from keyboard (will be overridden if laser steering is active)
            manual_pitch: Manual pitch input from keyboard
            manual_throttle: Manual throttle input from keyboard
            manual_yaw: Manual yaw input from keyboard
        
        Returns:
            tuple: (roll, pitch, throttle, yaw) control values
        """
        if not self.is_active:
            # Return manual control values unchanged
            return manual_roll, manual_pitch, manual_throttle, manual_yaw
        
        # Check if laser is detected
        if not self.laser_tracker.laser_detected:
            if time.time() - self.last_laser_time > self.laser_lost_timeout:
                projectairsim_log().warning("Laser lost for too long, stopping steering")
                self.stop()
                return manual_roll, manual_pitch, manual_throttle, manual_yaw
            # Laser lost but within timeout - return neutral controls but keep current throttle
            return 0, 0, manual_throttle, 0
        
        self.last_laser_time = time.time()
        
        # Get laser position relative to frame center
        if self.laser_tracker.laser_center is None or self.laser_tracker.frame_center is None:
            return 0, 0, manual_throttle, 0
        
        dx = self.laser_tracker.laser_center[0] - self.laser_tracker.frame_center[0]
        dy = self.laser_tracker.laser_center[1] - self.laser_tracker.frame_center[1]
        
        # Calculate control inputs using proportional control
        # Roll: When laser is to the right (dx > 0), we need to roll right (positive roll)
        roll = 0
        if abs(dx) > self.x_threshold:
            roll = self.kp_roll * (dx / 100.0) * 1000  # Scale to -1000 to +1000 range
            roll = max(-self.max_roll, min(self.max_roll, roll))
        
        # Pitch: When laser is below center (dy > 0 in image coords), we need to pitch forward (positive pitch)
        # Note: In image coordinates, y increases downward, so dy > 0 means laser is below center
        # To move forward towards the laser, we need positive pitch
        pitch = 0
        if abs(dy) > self.y_threshold:
            # Inverted: when laser is below center (dy > 0), pitch forward (positive pitch)
            pitch = self.kp_pitch * (dy / 100.0) * 1000
            pitch = max(-self.max_pitch, min(self.max_pitch, pitch))
        
        # Yaw: Optional - can be used to rotate towards laser if needed
        # For now, keep yaw neutral
        yaw = 0
        
        # Throttle: Keep current throttle value from manual control
        throttle = manual_throttle
        
        # Log control values (can be commented out for less verbose output)
        projectairsim_log().info(f"Laser steering - dx: {dx:.1f}, dy: {dy:.1f}, roll: {roll:.0f}, pitch: {pitch:.0f}")
        
        return int(roll), int(pitch), int(throttle), int(yaw)

