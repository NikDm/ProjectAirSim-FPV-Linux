"""
Copyright (C) Microsoft Corporation. All rights reserved.

Demonstrates flying a quadrotor drone with FPV camera and ball tracking capabilities.
Integrates ball tracking functionality from ball_tracking.py with FPV drone control.
Modified to automatically point and drive towards the green ball center.
"""

import asyncio
import cv2
import numpy as np
import imutils
from collections import deque
import time
import math

from projectairsim import ProjectAirSimClient, Drone, World
from projectairsim.utils import projectairsim_log
from projectairsim.image_utils import ImageDisplay
from projectairsim.drone import YawControlMode


class BallTracker:
    """Ball tracking class that processes camera frames for green ball detection."""
    
    def __init__(self, buffer_size=64, min_radius=2):
        # Define the lower and upper boundaries of the "green" ball in HSV color space
        # Green detection ranges - adjusted for better small ball detection
        self.green_lower = (35, 40, 40)   # Green hue range: 35-85, lower saturation/value for sensitivity
        self.green_upper = (85, 255, 255)   # Green saturation and value ranges
        
        self.pts = deque(maxlen=buffer_size)
        
        # Minimum radius for ball detection (lowered for small ball detection)
        self.min_radius = min_radius
        
        # FPS calculation variables
        self.fps_counter = 0
        self.fps_start_time = time.time()
        self.fps = 0
        
        # Ball tracking state
        self.ball_center = None
        self.frame_center = None
        self.ball_detected = False
        
    def process_frame(self, frame):
        """Process a single frame for ball tracking and return annotated frame."""
        if frame is None:
            return None, None, None
            
        # Calculate FPS
        self.fps_counter += 1
        if self.fps_counter >= 30:  # update FPS every 30 frames
            self.fps = self.fps_counter / (time.time() - self.fps_start_time)
            self.fps_counter = 0
            self.fps_start_time = time.time()
        
        # Resize the frame to preserve detail for small ball detection
        # Using larger width to maintain resolution for small objects
        frame = imutils.resize(frame, width=800)
        # Reduced blur kernel size to preserve small ball details
        blurred = cv2.GaussianBlur(frame, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        
        # Store frame center for ball position calculation
        self.frame_center = (frame.shape[1] // 2, frame.shape[0] // 2)
        
        # Construct a mask for the green color
        # Reduced erosion/dilation to preserve small ball detections
        mask = cv2.inRange(hsv, self.green_lower, self.green_upper)
        # Minimal morphological operations to avoid removing small balls
        mask = cv2.erode(mask, None, iterations=1)
        mask = cv2.dilate(mask, None, iterations=1)
        
        # Find contours in the mask and initialize the current
        # (x, y) center of the ball
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
            
            # Only proceed if the radius meets a minimum size (lowered for small ball detection)
            if radius > self.min_radius:
                # Update ball tracking state
                self.ball_center = center
                self.ball_detected = True
                
                # Draw the circle and centroid on the frame
                cv2.circle(frame, (int(x), int(y)), int(radius),
                    (0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)
                
                # Draw line from frame center to ball center
                cv2.line(frame, self.frame_center, center, (255, 0, 0), 2)
                
                # Log the ball coordinates to console
                # print(f"Green ball detected - x={center[0]}, y={center[1]}, radius={int(radius)}, FPS: {self.fps:.1f}")
            else:
                self.ball_detected = False
        else:
            self.ball_detected = False
        
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
        
        # Draw FPS and ball status on the frame
        cv2.putText(frame, f"FPS: {self.fps:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        status_text = "BALL DETECTED" if self.ball_detected else "NO BALL"
        cv2.putText(frame, status_text, (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0) if self.ball_detected else (0, 0, 255), 2)
        
        return frame, center, mask
    
    def get_ball_direction(self):
        """Calculate the direction from frame center to ball center."""
        if not self.ball_detected or self.ball_center is None or self.frame_center is None:
            return None
        
        # Calculate offset from frame center
        dx = self.ball_center[0] - self.frame_center[0]
        dy = self.ball_center[1] - self.frame_center[1]

        projectairsim_log().info(f"Ball direction - dx: {dx}, dy: {dy}")
        # Calculate angle (in radians) from frame center to ball
        angle = math.atan2(dx, dy)  # Using dy as reference for forward direction
        
        return angle


class BallFollowingController:
    """Improved controller for automatically following the ball with the drone."""
    
    def __init__(self, drone, ball_tracker):
        self.drone = drone
        self.ball_tracker = ball_tracker
        self.is_following = False
        self.last_ball_time = 0
        self.ball_lost_timeout = 2.0  # seconds to wait before stopping if ball is lost
        
        # PID-like control parameters for smooth movement
        self.kp_forward = 0.5    # Proportional gain for forward movement
        self.kp_lateral = 0.3   # Proportional gain for lateral movement (left/right)
        self.kp_vertical = 0.2   # Proportional gain for vertical movement
        
        # Speed limits
        self.max_forward_speed = 3.0   # m/s
        self.max_lateral_speed = 2.0   # m/s
        self.max_vertical_speed = 1.0  # m/s
        
        # Control thresholds
        self.x_threshold = 20  # pixels - smaller threshold for more responsive control
        self.y_threshold = 20  # pixels
        
    async def start_following(self):
        """Start the ball following behavior."""
        self.is_following = True
        projectairsim_log().info("Starting improved ball following mode")
        
        while self.is_following:
            try:
                await self._follow_ball_step()
                await asyncio.sleep(0.1)  # Control loop frequency
            except Exception as e:
                projectairsim_log().error(f"Error in ball following: {e}")
                await asyncio.sleep(0.1)
    
    async def stop_following(self):
        """Stop the ball following behavior."""
        self.is_following = False
        projectairsim_log().info("Stopping ball following mode")
    
    async def _follow_ball_step(self):
        """Execute one step of improved ball following control."""
        if not self.ball_tracker.ball_detected:
            if time.time() - self.last_ball_time > self.ball_lost_timeout:
                projectairsim_log().info("Ball lost for too long, stopping")
                await self.stop_following()
            return
        
        self.last_ball_time = time.time()
        
        # Get ball position relative to frame center
        dx = self.ball_tracker.ball_center[0] - self.ball_tracker.frame_center[0]
        dy = self.ball_tracker.ball_center[1] - self.ball_tracker.frame_center[1]
        
        # Calculate velocities using proportional control
        # Forward velocity: always move forward, but adjust based on ball distance
        forward_velocity = self.max_forward_speed * self.kp_forward
        
        # Lateral velocity: move left/right to center the ball
        lateral_velocity = 0.0
        if abs(dx) > self.x_threshold:
            # Proportional control: larger offset = faster correction
            # When ball is right (dx > 0), we want positive v_right to move right towards it
            lateral_velocity = self.kp_lateral * (dx / 100.0) * self.max_lateral_speed
            lateral_velocity = max(-self.max_lateral_speed, min(self.max_lateral_speed, lateral_velocity))
        
        # Vertical velocity: move up/down to center the ball vertically
        vertical_velocity = 0.0
        if abs(dy) > self.y_threshold:
            # Note: dy is positive when ball is below center (in image coordinates)
            # In drone body frame, positive Z is down, so we invert dy
            vertical_velocity = self.kp_vertical * (dy / 100.0) * self.max_vertical_speed
            vertical_velocity = max(-self.max_vertical_speed, min(self.max_vertical_speed, vertical_velocity))
        
        projectairsim_log().info(f"Ball control - dx: {dx}, dy: {dy}, velocities: forward={forward_velocity:.2f}, lateral={lateral_velocity:.2f}, vertical={vertical_velocity:.2f}")
        
        try:
            # Use body frame velocity control for smooth, continuous movement
            await self.drone.move_by_velocity_body_frame_async(
                v_forward=forward_velocity,
                v_right=lateral_velocity,
                v_down=vertical_velocity,
                duration=0.1,  # Short duration for continuous control
                yaw_control_mode=YawControlMode.MaxDegreeOfFreedom,
                yaw_is_rate=False,
                yaw=0.0  # Keep yaw stable, let velocity control handle movement
            )
                
        except Exception as e:
            projectairsim_log().error(f"Error applying control commands: {e}")


# Async main function to wrap async drone commands
async def main():
    # Create a Project AirSim client
    client = ProjectAirSimClient()

    # Initialize an ImageDisplay object to display camera sub-windows
    image_display = ImageDisplay()
    
    # Initialize ball tracker
    ball_tracker = BallTracker(buffer_size=64)

    try:
        # Connect to simulation environment
        client.connect()

        # Create a World object to interact with the sim world and load a scene
        # Note: To use Betaflight, you would need to:
        # 1. Complete the BetaflightApi implementation (create .cpp file)
        # 2. Register it in the controller factory
        # 3. Use "scene_basic_drone_betaflight_fpv.jsonc" instead
        world = World(client, "scene_basic_drone_fpv.jsonc", delay_after_load_sec=2)

        # Create a Drone object to interact with a drone in the loaded sim world
        drone = Drone(client, world, "Drone1")

        # ------------------------------------------------------------------------------

        # Subscribe to chase camera sensor as a client-side pop-up window
        chase_cam_window = "ChaseCam"
        image_display.add_chase_cam(chase_cam_window)
        client.subscribe(
            drone.sensors["Chase"]["scene_camera"],
            lambda _, chase: image_display.receive(chase, chase_cam_window),
        )

        # Subscribe to the FPV camera sensor's RGB images for ball tracking
        fpv_name = "FpvCamera"
        mask_name = "BallMask"
        image_display.add_image(fpv_name, subwin_idx=0)
        image_display.add_image(mask_name, subwin_idx=1)
        
        def process_fpv_frame(_, rgb):
            """Process FPV camera frame for ball tracking."""
            
            # Convert the image data to OpenCV format
            if rgb is not None and "data" in rgb and len(rgb["data"]) > 0:
               
                # Convert to numpy array and reshape
                frame = np.frombuffer(rgb["data"], dtype=np.uint8)
                frame = frame.reshape((rgb["height"], rgb["width"], 3))
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                
                # Process frame for ball tracking
                processed_frame, ball_center, mask = ball_tracker.process_frame(frame)
                
                if processed_frame is not None:
                    # Convert back to RGB for display
                    processed_frame = cv2.cvtColor(processed_frame, cv2.COLOR_BGR2RGB)
                    # Create a new image dict with processed data, preserving original structure
                    processed_rgb = rgb.copy()  # Copy all original fields
                    processed_rgb["data"] = processed_frame.tobytes()
                    processed_rgb["width"] = processed_frame.shape[1]
                    processed_rgb["height"] = processed_frame.shape[0]
                    # Ensure encoding field exists (defaults to RGB for 3-channel images)
                    if "encoding" not in processed_rgb:
                        processed_rgb["encoding"] = "RGB8"
                    image_display.receive(processed_rgb, fpv_name)
                
                # Display the mask
                if mask is not None:
                    # Convert mask to 3-channel for display
                    mask_display = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
                    # Create mask image dict
                    mask_rgb = rgb.copy()  # Copy all original fields
                    mask_rgb["data"] = mask_display.tobytes()
                    mask_rgb["width"] = mask_display.shape[1]
                    mask_rgb["height"] = mask_display.shape[0]
                    # Ensure encoding field exists
                    if "encoding" not in mask_rgb:
                        mask_rgb["encoding"] = "RGB8"
                    image_display.receive(mask_rgb, mask_name)
        
        client.subscribe(
            drone.sensors["FpvCamera"]["scene_camera"],
            process_fpv_frame,
        )

        image_display.start()

        # ------------------------------------------------------------------------------

        # Set the drone to be ready to fly
        drone.enable_api_control()
        drone.arm()

        # ------------------------------------------------------------------------------

        projectairsim_log().info("takeoff_async: starting")
        takeoff_task = (
            await drone.takeoff_async()
        )  # schedule an async task to start the command

        # Example 1: Wait on the result of async operation using 'await' keyword
        await takeoff_task
        projectairsim_log().info("takeoff_async: completed")

        # ------------------------------------------------------------------------------

        # Command the drone to move up in NED coordinate system at 1 m/s for 4 seconds
        # move_up_task = await drone.move_by_velocity_async(
        #     v_north=0.0, v_east=0.0, v_down=-1.0, duration=4.0
        # )
        # projectairsim_log().info("Move-Up invoked")

        # await move_up_task
        # projectairsim_log().info("Move-Up completed")

        # ------------------------------------------------------------------------------

        # Initialize improved ball following controller
        ball_controller = BallFollowingController(drone, ball_tracker)
        
        # Start ball following in a separate task
        following_task = asyncio.create_task(ball_controller.start_following())
        
        # Let the drone follow the ball for 60 seconds
        projectairsim_log().info("Starting improved ball following for 60 seconds...")
        await asyncio.sleep(60)
        
        # Stop ball following
        await ball_controller.stop_following()
        following_task.cancel()
        
        projectairsim_log().info("Ball following completed")

        # ------------------------------------------------------------------------------

        # Command the drone to land
        projectairsim_log().info("land_async: starting")
        land_task = await drone.land_async()
        await land_task
        projectairsim_log().info("land_async: completed")

        # ------------------------------------------------------------------------------

        # Shut down the drone
        drone.disarm()
        drone.disable_api_control()

        # ------------------------------------------------------------------------------

    # logs exception on the console
    except Exception as err:
        projectairsim_log().error(f"Exception occurred: {err}", exc_info=True)

    finally:
        # Always disconnect from the simulation environment to allow next connection
        client.disconnect()

        image_display.stop()


if __name__ == "__main__":
    asyncio.run(main())  # Runner for async main function