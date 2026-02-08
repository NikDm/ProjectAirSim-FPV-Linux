"""
Copyright (C) Microsoft Corporation. 
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.

Demonstrates flying a FastPhysics quadrotor using an ardupilot controller.

Note: Ardupilot controller also should be running for the Iris airframe 
      (FRAME_CLASS 1 and FRAME_TYPE 1) see Project AirSim docs for more info.

      This script uses pymavlink to connect to ArduPilot SITL and send control commands.
      Controls drone using MANUAL_CONTROL without GPS assistance.
"""

import asyncio
import cv2
import numpy as np
import os
import commentjson
import math

from projectairsim import ProjectAirSimClient, Drone, World
from projectairsim.utils import (
    projectairsim_log,
    convert_string_with_spaces_to_float_list,
    rpy_to_quaternion,
)
from projectairsim.image_utils import ImageDisplay
from projectairsim.types import Pose, Vector3, Quaternion

# Import control classes from separate module
from ardupilot_controls import ArduPilotController, KeyboardController, KEYBOARD_AVAILABLE
from laser_tracker import LaserTracker
from laser_steering import LaserSteeringController


def get_starting_pose_from_scene_config(scene_config_name: str, drone_name: str = "Drone1", sim_config_path: str = "sim_config/"):
    """
    Parse the scene configuration file to get the starting pose of the drone.
    
    Args:
        scene_config_name: Name of the scene config file (e.g., "scene_ardu_quadrotor.jsonc")
        drone_name: Name of the drone actor in the scene
        sim_config_path: Path to the sim_config directory
        
    Returns:
        Pose: The starting pose of the drone, or None if not found
    """
    try:
        config_path = os.path.join(sim_config_path, scene_config_name)
        with open(config_path, 'r') as f:
            scene_config = commentjson.load(f)
        
        # Find the drone actor in the actors list
        if "actors" in scene_config:
            for actor in scene_config["actors"]:
                if actor.get("type") == "robot" and actor.get("name") == drone_name:
                    origin = actor.get("origin", {})
                    
                    # Parse xyz position
                    if "xyz" in origin:
                        x, y, z = convert_string_with_spaces_to_float_list(origin["xyz"])
                    else:
                        projectairsim_log().error(f"No 'xyz' found in origin for {drone_name}")
                        return None
                    
                    # Parse rotation (rpy-deg or rpy)
                    roll, pitch, yaw = 0.0, 0.0, 0.0
                    if "rpy-deg" in origin:
                        roll, pitch, yaw = convert_string_with_spaces_to_float_list(origin["rpy-deg"])
                        # Convert degrees to radians
                        roll = math.radians(roll)
                        pitch = math.radians(pitch)
                        yaw = math.radians(yaw)
                    elif "rpy" in origin:
                        roll, pitch, yaw = convert_string_with_spaces_to_float_list(origin["rpy"])
                    
                    # Convert RPY to quaternion
                    quat_w, quat_x, quat_y, quat_z = rpy_to_quaternion(roll, pitch, yaw)
                    
                    # Create Pose object
                    translation = Vector3({"x": x, "y": y, "z": z})
                    rotation = Quaternion({"w": quat_w, "x": quat_x, "y": quat_y, "z": quat_z})
                    pose = Pose({
                        "translation": translation,
                        "rotation": rotation,
                        "frame_id": "DEFAULT_ID"
                    })
                    
                    projectairsim_log().info(f"Loaded starting pose for {drone_name}: xyz=({x}, {y}, {z}), rpy=({math.degrees(roll):.1f}°, {math.degrees(pitch):.1f}°, {math.degrees(yaw):.1f}°)")
                    return pose
        
        projectairsim_log().error(f"Drone '{drone_name}' not found in scene config")
        return None
        
    except Exception as e:
        projectairsim_log().error(f"Failed to load starting pose from scene config: {e}")
        return None


# Async main function to wrap async drone commands
async def main():
    # Create a Project AirSim client
    client = ProjectAirSimClient()

    # Initialize an ImageDisplay object to position up to 2 pop-up sub-windows
    image_display = ImageDisplay()
    
    # Initialize laser tracker
    laser_tracker = LaserTracker(buffer_size=64)

    try:
        # Connect to simulation environment
        client.connect()

        # Create a World object to interact with the sim world and load a scene
        scene_config_name = "scene_ardu_quadrotor.jsonc"
        world = World(client, scene_config_name, delay_after_load_sec=2)

        # Create a Drone object to interact with a drone in the loaded sim world
        drone = Drone(client, world, "Drone1")
        
        # Load the starting pose from the scene config for reset functionality
        scene_config_name = "scene_ardu_quadrotor.jsonc"
        starting_pose = get_starting_pose_from_scene_config(scene_config_name, "Drone1", "sim_config/")

    # ------------------------------------------------------------------------------
    # Subscribe to chase camera sensor
        chase_cam_window = "ChaseCam"
        image_display.add_chase_cam(chase_cam_window)
        client.subscribe(
            drone.sensors["Chase"]["scene_camera"],
            lambda _, chase: image_display.receive(chase, chase_cam_window),
        )

        # Subscribe to the FPV camera sensor's RGB images for laser tracking
        fpv_name = "FpvCamera"
        mask_name = "LaserMask"
        image_display.add_image(fpv_name, subwin_idx=0)
        image_display.add_image(mask_name, subwin_idx=1)
        
        def process_fpv_frame(_, rgb):
            """Process FPV camera frame for laser tracking."""
            
            # Convert the image data to OpenCV format
            if rgb is not None and "data" in rgb and len(rgb["data"]) > 0:
               
                # Convert to numpy array and reshape
                frame = np.frombuffer(rgb["data"], dtype=np.uint8)
                frame = frame.reshape((rgb["height"], rgb["width"], 3))
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                
                # Process frame for laser tracking
                processed_frame, laser_center, mask = laser_tracker.process_frame(frame)
                
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
        # Connect to ArduPilot SITL using pymavlink
        ardu_controller = ArduPilotController(connection_string='udp:127.0.0.1:14550')
        
        if not ardu_controller.connect():
            projectairsim_log().error("Failed to connect to ArduPilot SITL. Make sure SITL is running.")
            projectairsim_log().info("Waiting for camera feed...")
            input("Press any key to stop...")
        else:
            # Wait a moment for everything to stabilize
            await asyncio.sleep(2)
            
            # Initialize keyboard controller
            kb_controller = KeyboardController(ardu_controller)
            
            # Initialize laser steering controller
            laser_steering = LaserSteeringController(ardu_controller, laser_tracker)
            
            # Set to STABILIZE mode (attitude control, no GPS needed)
            current_mode = 'STABILIZE'
            ardu_controller.set_mode(current_mode)
            await asyncio.sleep(1)
            
            # Start keyboard control
            if not KEYBOARD_AVAILABLE:
                projectairsim_log().error("pynput library not available. Install with: pip install pynput")
                projectairsim_log().info("Falling back to manual input mode...")
                input("Press Enter to arm the vehicle...")
                if ardu_controller.arm():
                    input("Press Enter to disarm...")
                    ardu_controller.disarm()
            else:
                kb_controller.start()
                is_armed = False
                running = True
                current_mode = 'STABILIZE'
                laser_steering_active = False
                
                projectairsim_log().info("Keyboard control active. Use keys to control the drone.")
                projectairsim_log().info("Press 'T' to arm/disarm, 'F' to toggle laser steering, 'H' for help, 'ESC' to quit.")
                
                try:
                    # Main control loop - send commands at ~20Hz
                    while running:
                        
                        # Get manual control state from keyboard controller
                        manual_state = kb_controller.get_state()
                        
                        # Convert manual state to control inputs
                        # Roll: -30° to +30° -> -1000 to +1000
                        manual_roll = int((manual_state['roll'] / 30.0) * 1000)
                        manual_roll = max(-1000, min(1000, manual_roll))
                        
                        # Pitch: -30° to +30° -> -1000 to +1000
                        manual_pitch = int((manual_state['pitch'] / 30.0) * 1000)
                        manual_pitch = max(-1000, min(1000, manual_pitch))
                        
                        # Yaw: normalize to -1000 to +1000
                        manual_yaw = int((manual_state['yaw'] / 30.0) * 1000)
                        manual_yaw = max(-1000, min(1000, manual_yaw))
                        
                        # Throttle: 0.0 to 1.0 -> 0 to 1000
                        manual_throttle = int(manual_state['thrust'] * 1000)
                        manual_throttle = max(0, min(1000, manual_throttle))
                        
                        # Apply laser steering if active (overrides manual control)
                        roll, pitch, throttle, yaw = laser_steering.update(
                            manual_roll, manual_pitch, manual_throttle, manual_yaw
                        )
                        
                        # Send control command to ArduPilot
                        ardu_controller.set_manual_control(
                            pitch=pitch,
                            roll=roll,
                            throttle=throttle,
                            yaw=yaw
                        )
                        
                        # Check for commands from keyboard
                        command = kb_controller.get_command(timeout=0.05)
                        if command == 'toggle_arm':
                            if is_armed:
                                projectairsim_log().info("Disarming...")
                                if ardu_controller.disarm():
                                    is_armed = False
                            else:
                                projectairsim_log().info("Arming...")
                                if ardu_controller.arm():
                                    is_armed = True
                                    await asyncio.sleep(1)
                        elif command == 'land':
                            projectairsim_log().info("Switching to LAND mode...")
                            ardu_controller.set_mode('LAND')
                        elif command == 'toggle_mode':
                            if current_mode == 'STABILIZE':
                                current_mode = 'ALT_HOLD'
                                projectairsim_log().info("Switching to ALT_HOLD mode...")
                            else:
                                current_mode = 'STABILIZE'
                                projectairsim_log().info("Switching to STABILIZE mode...")
                            ardu_controller.set_mode(current_mode)
                        elif command == 'toggle_laser_steering':
                            if laser_steering_active:
                                laser_steering.stop()
                                laser_steering_active = False
                                projectairsim_log().info("Laser steering OFF - Manual control active")
                            else:
                                laser_steering.start()
                                laser_steering_active = True
                                projectairsim_log().info("Laser steering ON - Auto-tracking laser point")
                        elif command == 'reset_position':
                            if starting_pose is not None:
                                projectairsim_log().info("Resetting drone to starting position...")
                                # Reset kinematics (velocity, etc.) when resetting position
                                success = drone.set_pose(starting_pose, reset_kinematics=True)
                                if success:
                                    projectairsim_log().info("Drone reset to starting position successfully")
                                else:
                                    projectairsim_log().error("Failed to reset drone position")
                            else:
                                projectairsim_log().error("Starting pose not available. Cannot reset position.")
                        elif command == 'quit':
                            running = False
                            break
                        
                        await asyncio.sleep(0.05)  # ~20Hz control rate
                        
                except KeyboardInterrupt:
                    projectairsim_log().info("Interrupted by user")
                    running = False
                
                # Cleanup
                kb_controller.stop()
                
                if is_armed:
                    projectairsim_log().info("Disarming before exit...")
                    ardu_controller.disarm()
            
            projectairsim_log().info("Control session ended. Waiting for user input...")
            input("Press any key to stop seeing the drone's camera images...")
            
            ardu_controller.disconnect()

        # ------------------------------------------------------------------------------

    except Exception as err:
        projectairsim_log().error(f"Exception occurred: {err}", exc_info=True)

    finally:
        # Always disconnect from the simulation environment to allow next connection
        client.disconnect()
        image_display.stop()


if __name__ == "__main__":
    asyncio.run(main())  # Runner for async main function