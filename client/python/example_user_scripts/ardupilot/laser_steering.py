"""
Copyright (C) Microsoft Corporation.
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.

Laser steering controller for automatically keeping laser point centered and flying towards it.
Enhanced with PID control, filtering, and rate limiting for stable, smooth tracking.
"""

import time
from projectairsim.utils import projectairsim_log


class PIDController:
    """Single-axis PID controller with anti-windup."""

    def __init__(self, kp, ki, kd, output_min, output_max):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max

        # State variables
        self.prev_error = 0.0
        self.integral = 0.0
        self.integral_max = 100.0  # Anti-windup limit
        self.prev_time = None

    def update(self, error, current_time):
        """
        Calculate PID output with anti-windup.

        Args:
            error: Current error (setpoint - measurement)
            current_time: Current timestamp in seconds

        Returns:
            control_output: PID output value
        """
        # Initialize timing on first call
        if self.prev_time is None:
            self.prev_time = current_time
            self.prev_error = error
            return 0.0

        # Calculate dt
        dt = current_time - self.prev_time
        if dt <= 0:
            dt = 0.01  # Prevent division by zero

        # Proportional term
        p_term = self.kp * error

        # Integral term with anti-windup
        self.integral += error * dt
        self.integral = max(-self.integral_max, min(self.integral_max, self.integral))
        i_term = self.ki * self.integral

        # Derivative term
        derivative = (error - self.prev_error) / dt
        d_term = self.kd * derivative

        # Calculate output
        output = p_term + i_term + d_term

        # Clamp output
        output = max(self.output_min, min(self.output_max, output))

        # Update state
        self.prev_error = error
        self.prev_time = current_time

        return output

    def reset(self):
        """Reset controller state."""
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = None


class ExponentialFilter:
    """Exponential moving average filter for smoothing noisy signals."""

    def __init__(self, alpha=0.3):
        """
        Initialize filter.

        Args:
            alpha: Smoothing factor (0-1). Lower = more smoothing.
                   0.1 = heavy smoothing (90% history, 10% new)
                   0.5 = balanced
                   0.9 = light smoothing (10% history, 90% new)
        """
        self.alpha = alpha
        self.filtered_value = None

    def update(self, new_value):
        """Update filter with new measurement."""
        if self.filtered_value is None:
            self.filtered_value = new_value
        else:
            self.filtered_value = self.alpha * new_value + (1 - self.alpha) * self.filtered_value
        return self.filtered_value

    def reset(self):
        """Reset filter state."""
        self.filtered_value = None


class LaserSteeringController:
    """Enhanced controller with PID control and filtering for stable laser tracking."""

    def __init__(self, ardu_controller, laser_tracker):
        self.ardu_controller = ardu_controller
        self.laser_tracker = laser_tracker
        self.is_active = False
        self.last_laser_time = 0
        self.laser_lost_timeout = 2.0  # seconds to wait before stopping if laser is lost

        # TUNED PID PARAMETERS
        # These are conservative values for stable convergence

        # Pitch axis (forward/backward) - More aggressive since it's the main control
        pitch_kp = 0.08   # Reduced from 0.3 to prevent overshooting
        pitch_ki = 0.005  # Small integral for steady-state error elimination
        pitch_kd = 0.15   # Derivative for damping (should be ~2x Kp)

        # Roll axis (left/right) - Similar to pitch
        roll_kp = 0.08
        roll_ki = 0.005
        roll_kd = 0.15

        # Control limits (REDUCED for safety)
        self.max_pitch = 150   # Reduced from 300 (15% authority)
        self.max_roll = 150    # Reduced from 300

        # Initialize PID controllers
        self.pitch_pid = PIDController(
            kp=pitch_kp, ki=pitch_ki, kd=pitch_kd,
            output_min=-self.max_pitch, output_max=self.max_pitch
        )
        self.roll_pid = PIDController(
            kp=roll_kp, ki=roll_ki, kd=roll_kd,
            output_min=-self.max_roll, output_max=self.max_roll
        )

        # Exponential filters for position smoothing
        self.dx_filter = ExponentialFilter(alpha=0.4)  # 40% new, 60% history
        self.dy_filter = ExponentialFilter(alpha=0.4)

        # Control thresholds (deadzones) - INCREASED for stability
        self.x_threshold = 30  # pixels (up from 20)
        self.y_threshold = 30  # pixels (up from 20)

        # Rate limiting (prevents sudden control jumps)
        self.max_control_change_per_update = 50  # max change in control per cycle
        self.prev_roll = 0
        self.prev_pitch = 0

    def start(self):
        """Start laser steering mode."""
        self.is_active = True
        self.pitch_pid.reset()
        self.roll_pid.reset()
        self.dx_filter.reset()
        self.dy_filter.reset()
        self.prev_roll = 0
        self.prev_pitch = 0
        projectairsim_log().info("Laser steering mode activated with PID control")

    def stop(self):
        """Stop laser steering mode."""
        self.is_active = False
        projectairsim_log().info("Laser steering mode deactivated")

    def _apply_rate_limit(self, new_value, prev_value, max_change):
        """Apply rate limiting to prevent sudden control changes."""
        delta = new_value - prev_value
        if abs(delta) > max_change:
            return prev_value + (max_change if delta > 0 else -max_change)
        return new_value

    def update(self, manual_roll=0, manual_pitch=0, manual_throttle=500, manual_yaw=0):
        """
        Update control based on laser position using PID control.

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
        current_time = time.time()

        # Get laser position relative to frame center
        if self.laser_tracker.laser_center is None or self.laser_tracker.frame_center is None:
            return 0, 0, manual_throttle, 0

        # Calculate raw errors
        dx_raw = self.laser_tracker.laser_center[0] - self.laser_tracker.frame_center[0]
        dy_raw = self.laser_tracker.laser_center[1] - self.laser_tracker.frame_center[1]

        # Apply exponential filtering to smooth noisy measurements
        dx = self.dx_filter.update(dx_raw)
        dy = self.dy_filter.update(dy_raw)

        # Apply deadzones (don't correct small errors)
        dx_error = dx if abs(dx) > self.x_threshold else 0.0
        dy_error = dy if abs(dy) > self.y_threshold else 0.0

        # Calculate PID outputs (errors are in pixels, scaled to control range)
        # Scale factor: 100 pixels = full error scale
        roll = self.roll_pid.update(dx_error / 100.0, current_time) * 1000
        pitch = self.pitch_pid.update(dy_error / 100.0, current_time) * 1000

        # Apply rate limiting to prevent jerky motion
        roll = self._apply_rate_limit(roll, self.prev_roll, self.max_control_change_per_update)
        pitch = self._apply_rate_limit(pitch, self.prev_pitch, self.max_control_change_per_update)

        # Store for next iteration
        self.prev_roll = roll
        self.prev_pitch = pitch

        # Yaw and throttle remain manual
        yaw = 0
        throttle = manual_throttle

        # Enhanced logging with filtered values (for tuning)
        projectairsim_log().info(
            f"Laser PID - dx_raw: {dx_raw:.1f}, dy_raw: {dy_raw:.1f}, "
            f"dx_filt: {dx:.1f}, dy_filt: {dy:.1f}, "
            f"roll: {roll:.0f}, pitch: {pitch:.0f}"
        )

        return int(roll), int(pitch), int(throttle), int(yaw)
