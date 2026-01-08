"""
Copyright (C) Microsoft Corporation. 
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.

ArduPilot control classes for MAVLink communication and keyboard input.

This module provides:
- ArduPilotController: MAVLink commands to ArduPilot SITL
- KeyboardController: Keyboard input handler for drone control
"""

import time
import threading
import queue
from pymavlink import mavutil

from projectairsim.utils import projectairsim_log

# Try to import pynput library for keyboard input
try:
    from pynput import keyboard
    KEYBOARD_AVAILABLE = True
except ImportError:
    KEYBOARD_AVAILABLE = False
    projectairsim_log().warning("pynput library not available. Install with: pip install pynput")


class ArduPilotController:
    """Helper class to send MAVLink commands to ArduPilot SITL."""
    
    def __init__(self, connection_string='udp:127.0.0.1:14550'):
        """
        Initialize connection to ArduPilot SITL.
        
        Args:
            connection_string: MAVLink connection string (default: 'udp:127.0.0.1:14550')
                              Other options: 'udpin:0.0.0.0:14550' (listen), 'tcp:127.0.0.1:5760'
        """
        self.connection_string = connection_string
        self.master = None
        
    def connect(self):
        """Connect to ArduPilot SITL."""
        try:
            projectairsim_log().info(f"Connecting to ArduPilot SITL at {self.connection_string}...")
            self.master = mavutil.mavlink_connection(self.connection_string)
            self.master.wait_heartbeat()
            projectairsim_log().info(f"Connected! Heartbeat from system {self.master.target_system}, component {self.master.target_component}")
            return True
        except Exception as e:
            projectairsim_log().error(f"Failed to connect to ArduPilot: {e}")
            return False
    
    def disconnect(self):
        """Close connection to ArduPilot."""
        if self.master:
            self.master.close()
            self.master = None
    
    def arm(self):
        """Arm the vehicle."""
        if not self.master:
            projectairsim_log().error("Not connected to ArduPilot")
            return False
        
        projectairsim_log().info("Arming vehicle...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0  # 1 = arm
        )
        
        # Wait for arm acknowledgment
        timeout = time.time() + 5  # 5 second timeout
        while time.time() < timeout:
            msg = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=1)
            if msg:
                if msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                    if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                        projectairsim_log().info("Vehicle armed successfully")
                        return True
                    else:
                        projectairsim_log().error(f"Arming failed: {msg.result}")
                        return False
        projectairsim_log().warning("Arming acknowledgment timeout")
        return False
    
    def disarm(self):
        """Disarm the vehicle."""
        if not self.master:
            projectairsim_log().error("Not connected to ArduPilot")
            return False
        
        projectairsim_log().info("Disarming vehicle...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0, 0, 0, 0, 0, 0, 0  # 0 = disarm
        )
        
        # Wait for disarm acknowledgment
        msg = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        if msg and msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                projectairsim_log().info("Vehicle disarmed successfully")
                return True
        return False
    
    def set_mode(self, mode):
        """
        Set flight mode.
        
        Args:
            mode: Flight mode string (e.g., 'STABILIZE', 'ACRO', 'ALT_HOLD', 'LAND')
        """
        if not self.master:
            projectairsim_log().error("Not connected to ArduPilot")
            return False
        
        # Get mode ID
        mode_id = self.master.mode_mapping().get(mode)
        if mode_id is None:
            projectairsim_log().error(f"Unknown mode: {mode}")
            return False
        
        projectairsim_log().info(f"Setting mode to {mode}...")
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        
        # Wait a bit and check if mode changed
        time.sleep(0.5)
        return True

    def set_manual_control(self, pitch=0, roll=0, throttle=500, yaw=0):
        """
        Send MANUAL_CONTROL command (RC-like control, no GPS required).
        
        According to ArduPilot docs: https://ardupilot.org/dev/docs/mavlink-rcinput.html#manual-control
        
        Args:
            pitch: Pitch input -1000 (backwards) to +1000 (forwards)
            roll: Roll input -1000 (left) to +1000 (right)
            throttle: Throttle input 0 (down) to +1000 (up)
            yaw: Yaw input -1000 (counter-clockwise) to +1000 (clockwise)
        """
        if not self.master:
            projectairsim_log().error("Not connected to ArduPilot")
            return False
        
        # Clamp values to valid range
        pitch = max(-1000, min(1000, int(pitch)))
        roll = max(-1000, min(1000, int(roll)))
        throttle = max(0, min(1000, int(throttle)))
        yaw = max(-1000, min(1000, int(yaw)))
        
        # Send MANUAL_CONTROL message
        # Note: According to MAVLink spec, the order is: target_system, x(pitch), y(roll), z(throttle), r(yaw), buttons
        self.master.mav.manual_control_send(
            self.master.target_system,  # target
            pitch,   # x (pitch)
            roll,    # y (roll)
            throttle,  # z (throttle)
            yaw,     # r (yaw)
            0        # buttons (unused)
        )
        return True
    
    def land(self):
        """Land the vehicle."""
        if not self.master:
            projectairsim_log().error("Not connected to ArduPilot")
            return False
        
        projectairsim_log().info("Landing...")
        return self.set_mode('LAND')
    
    def get_heartbeat(self):
        """Get latest heartbeat message."""
        if not self.master:
            return None
        return self.master.recv_match(type='HEARTBEAT', blocking=False)


class KeyboardController:
    """Keyboard input handler for drone control."""
    
    def __init__(self, ardu_controller):
        self.ardu_controller = ardu_controller
        self.running = False
        self.listener = None
        
        # Control state
        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        self.yaw_deg = 0.0
        self.thrust = 0.5  # Default hover thrust
        self.max_angle = 30.0  # Maximum roll/pitch angle in degrees
        self.angle_step = 2.0  # Angle change per key press
        self.thrust_step = 0.05  # Thrust change per key press
        
        # Command queue for async communication
        self.command_queue = queue.Queue()
        
        # Track pressed keys for continuous control
        self.pressed_keys = set()
        
        # Control lock
        self.lock = threading.Lock()
    
    def print_controls(self):
        """Print keyboard control instructions."""
        print("\n" + "="*60)
        print("KEYBOARD CONTROLS")
        print("="*60)
        print("Movement:")
        print("  W/S     - Pitch forward/backward")
        print("  A/D     - Roll left/right")
        print("  Q/E     - Yaw left/right")
        print("  ↑/↓     - Increase/decrease thrust")
        print("  Space   - Reset to level (roll=0, pitch=0, yaw=0)")
        print("\nCommands:")
        print("  T       - Arm/Disarm")
        print("  L       - Land mode")
        print("  M       - Toggle STABILIZE/ALT_HOLD mode")
        print("  F       - Toggle laser steering mode")
        print("  0-9     - Set thrust to 0.0-0.9")
        print("  R       - Reset all controls to neutral")
        print("  H       - Show this help")
        print("  ESC     - Quit")
        print("\nCurrent State:")
        print(f"  Thrust: {self.thrust:.2f} (0.0-1.0)")
        print(f"  Roll: {self.roll_deg:.1f}°")
        print(f"  Pitch: {self.pitch_deg:.1f}°")
        print(f"  Yaw: {self.yaw_deg:.1f}°")
        print("="*60 + "\n")
    
    def update_control(self, lock_acquired=False):
        """
        Update control values and send to ArduPilot using MANUAL_CONTROL.
        
        Args:
            lock_acquired: If True, assumes lock is already held and doesn't acquire it again
        """
        # Convert angle-based control to normalized values (-1000 to +1000)
        # Roll: -30° to +30° -> -1000 to +1000
        roll_normalized = int((self.roll_deg / self.max_angle) * 1000)
        roll_normalized = max(-1000, min(1000, roll_normalized))
        
        # Pitch: -30° to +30° -> -1000 to +1000
        pitch_normalized = int((self.pitch_deg / self.max_angle) * 1000)
        pitch_normalized = max(-1000, min(1000, pitch_normalized))
        
        # Yaw: limit to reasonable range and normalize
        # Assuming max yaw rate equivalent to max angle
        yaw_normalized = int((self.yaw_deg / self.max_angle) * 1000)
        yaw_normalized = max(-1000, min(1000, yaw_normalized))
        
        # Thrust: 0.0 to 1.0 -> 0 to 1000
        throttle_normalized = int(self.thrust * 1000)
        throttle_normalized = max(0, min(1000, throttle_normalized))
        
        if lock_acquired:
            # Lock already held, don't acquire again
            self.ardu_controller.set_manual_control(
                pitch=pitch_normalized,
                roll=roll_normalized,
                throttle=throttle_normalized,
                yaw=yaw_normalized
            )
        else:
            # Acquire lock
            with self.lock:
                self.ardu_controller.set_manual_control(
                    pitch=pitch_normalized,
                    roll=roll_normalized,
                    throttle=throttle_normalized,
                    yaw=yaw_normalized
                )
    
    def _get_key_name(self, key):
        """Convert pynput key to string name."""
        try:
            # Check if it's a special key (has name attribute)
            if hasattr(key, 'name') and key.name:
                key_name = key.name.lower()
                # Normalize arrow key names
                if key_name == 'up':
                    return 'up'
                elif key_name == 'down':
                    return 'down'
                elif key_name == 'left':
                    return 'left'
                elif key_name == 'right':
                    return 'right'
                return key_name
            # Check if it's a regular character key
            elif hasattr(key, 'char') and key.char:
                char = key.char.lower()
                # Check for escape sequences (arrow keys sometimes come through as chars)
                if char == '\x1b':  # ESC character
                    return 'esc'
                return char
            else:
                # Fallback to string representation
                key_str = str(key)
                # Remove 'Key.' prefix if present
                if key_str.startswith('Key.'):
                    key_name = key_str[4:].lower()
                    # Normalize arrow key names
                    if 'up' in key_name:
                        return 'up'
                    elif 'down' in key_name:
                        return 'down'
                    elif 'left' in key_name:
                        return 'left'
                    elif 'right' in key_name:
                        return 'right'
                    return key_name
                return key_str.lower()
        except Exception as e:
            projectairsim_log().debug(f"Error getting key name: {e}, key type: {type(key)}")
            return str(key).lower()
    
    def _on_press(self, key):
        """Handle key press events using pynput library."""
        if not self.running:
            return False
        
        try:
            key_name = self._get_key_name(key)
            # Debug output - uncomment to see all detected keys
            # print(f"DEBUG: Key detected - name: '{key_name}', type: {type(key)}")
            
            # Handle the key
            self._handle_key(key_name)
            
            # Return True to continue listening
            # Note: suppress is set in Listener, but may not work on all systems
        except Exception as e:
            projectairsim_log().error(f"Error handling key press: {e}")
            import traceback
            traceback.print_exc()
        
        return True
    
    def _on_release(self, key):
        """Handle key release events."""
        if not self.running:
            return False
        
        try:
            key_name = self._get_key_name(key)
            self.pressed_keys.discard(key_name)
            
            # Stop listener only on ESC release (not 'q' since it's used for yaw)
            if key_name == 'esc' or key_name == 'escape':
                return False
        except:
            pass
        
        return True
    
    def get_command(self, timeout=0.1):
        """Get next command from queue (non-blocking)."""
        try:
            return self.command_queue.get(timeout=timeout)
        except queue.Empty:
            return None
    
    def start(self):
        """Start keyboard control loop."""
        if not KEYBOARD_AVAILABLE:
            projectairsim_log().error("pynput library not available. Cannot start keyboard control.")
            return False
        
        self.running = True
        self.print_controls()
        
        # Use pynput library
        try:
            # Note: suppress=True may not work perfectly on all Linux systems
            # Keys might still appear in terminal, but they will be detected
            self.listener = keyboard.Listener(
                on_press=self._on_press,
                on_release=self._on_release,
                suppress=False  # Don't suppress keys - allow normal keyboard behavior
            )
            self.listener.start()
            
            time.sleep(0.2)
            
            if not self.listener.is_alive():
                projectairsim_log().error("Keyboard listener failed to start!")
                return False
                
            projectairsim_log().info("Keyboard control started using 'pynput' library.")
            projectairsim_log().warning("NOTE: On Linux, keys may still appear in terminal but will be detected.")
            return True
        except Exception as e:
            projectairsim_log().error(f"Failed to start keyboard listener: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def _handle_key(self, key_name):
        """Handle key press - common logic for both libraries."""
        self.pressed_keys.add(key_name)
        
        with self.lock:
            if key_name == 'w':  # Pitch forward
                self.pitch_deg = min(self.max_angle, self.pitch_deg + self.angle_step)
                print(f"Pitch: {self.pitch_deg:.1f}°")
                self.update_control(lock_acquired=True)
            elif key_name == 's':  # Pitch backward
                self.pitch_deg = max(-self.max_angle, self.pitch_deg - self.angle_step)
                print(f"Pitch: {self.pitch_deg:.1f}°")
                self.update_control(lock_acquired=True)
            elif key_name == 'a':  # Roll left
                self.roll_deg = max(-self.max_angle, self.roll_deg - self.angle_step)
                print(f"Roll: {self.roll_deg:.1f}°")
                self.update_control(lock_acquired=True)
            elif key_name == 'd':  # Roll right
                self.roll_deg = min(self.max_angle, self.roll_deg + self.angle_step)
                print(f"Roll: {self.roll_deg:.1f}°")
                self.update_control(lock_acquired=True)
            elif key_name == 'q':  # Yaw left
                self.yaw_deg -= self.angle_step
                print(f"Yaw: {self.yaw_deg:.1f}°")
                self.update_control(lock_acquired=True)
            elif key_name == 'e':  # Yaw right
                self.yaw_deg += self.angle_step
                print(f"Yaw: {self.yaw_deg:.1f}°")
                self.update_control(lock_acquired=True)
            elif key_name in ['up', 'page up']:  # Increase thrust
                self.thrust = min(100.0, self.thrust + self.thrust_step)
                print(f"Thrust: {self.thrust:.2f}")
                self.update_control(lock_acquired=True)
            elif key_name in ['down', 'page down']:  # Decrease thrust
                self.thrust = max(0.0, self.thrust - self.thrust_step)
                print(f"Thrust: {self.thrust:.2f}")
                self.update_control(lock_acquired=True)
            elif key_name == 'space':  # Reset to level
                self.roll_deg = 0.0
                self.pitch_deg = 0.0
                self.yaw_deg = 0.0
                print("Reset to level")
                self.update_control(lock_acquired=True)
            elif key_name == 'r':  # Reset all
                self.roll_deg = 0.0
                self.pitch_deg = 0.0
                self.yaw_deg = 0.0
                self.thrust = 0.5
                print("Reset all controls")
                self.update_control(lock_acquired=True)
            elif key_name == 't':  # Toggle arm/disarm
                print("Toggling arm/disarm...")
                self.command_queue.put('toggle_arm')
            elif key_name == 'l':  # Land
                print("Landing...")
                self.command_queue.put('land')
            elif key_name == 'm':  # Toggle mode
                print("Toggling mode...")
                self.command_queue.put('toggle_mode')
            elif key_name == 'f':  # Toggle laser steering
                print("Toggling laser steering...")
                self.command_queue.put('toggle_laser_steering')
            elif key_name == 'h':  # Help
                self.print_controls()
            elif key_name in ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9']:
                self.thrust = int(key_name) / 10.0
                print(f"Thrust set to: {self.thrust:.1f}")
                self.update_control(lock_acquired=True)
            elif key_name in ['esc', 'escape']:  # Quit
                print("Quitting...")
                self.command_queue.put('quit')
    
    def stop(self):
        """Stop keyboard control."""
        self.running = False
        if self.listener:
            self.listener.stop()
            self.listener = None
    
    def get_state(self):
        """Get current control state."""
        with self.lock:
            return {
                'roll': self.roll_deg,
                'pitch': self.pitch_deg,
                'yaw': self.yaw_deg,
                'thrust': self.thrust
            }

