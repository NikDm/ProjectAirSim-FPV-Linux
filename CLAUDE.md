# ProjectAirSim - Autonomous Laser-Tracking Drone

## Project Overview

This project implements an autonomous quadrotor drone that can follow a green laser dot using vision-based tracking in AirSim simulation with ArduPilot SITL.

**Key Features:**
- Manual keyboard control of quadrotor
- Real-time green laser detection using HSV color filtering
- Autonomous laser tracking with PID control
- Toggle between manual and autonomous modes (F key)
- Software-in-the-Loop simulation with realistic physics

## Architecture

### Core Components

1. **Simulation Environment**
   - AirSim with Unreal Engine 5.2 (Blocks map)
   - ArduPilot SITL (Software-in-the-Loop)
   - Physics simulation at 333 Hz (3ms steps)

2. **Vision System** ([laser_tracker.py](client/python/example_user_scripts/ardupilot/laser_tracker.py))
   - HSV color space filtering for green laser detection (Hue: 35-85°)
   - Processes FPV camera at ~30 FPS (400x225 resolution, resized to 800px width)
   - Provides laser position relative to frame center

3. **Control System** ([laser_steering.py](client/python/example_user_scripts/ardupilot/laser_steering.py))
   - PID controller for stable, smooth tracking
   - Exponential filtering for noise reduction
   - Rate limiting to prevent jerky motion
   - Converts pixel errors to MAVLink MANUAL_CONTROL commands

4. **Interface** ([ardupilot_controls.py](client/python/example_user_scripts/ardupilot/ardupilot_controls.py))
   - ArduPilotController: MAVLink communication via pymavlink
   - KeyboardController: Real-time keyboard input via pynput
   - Key bindings for arm/disarm, mode switching, laser steering toggle

5. **Main Application** ([ardupilot_quadrotor.py](client/python/example_user_scripts/ardupilot/ardupilot_quadrotor.py))
   - Async event loop integrating all components
   - 20 Hz control loop
   - Camera frame processing and display

## PID Control System

### Problem Solved (February 2026)

The original implementation used pure proportional control with aggressive gains, causing:
- Severe oscillations and bouncing behavior
- Diverging errors (growing from 3→216+ pixels)
- Control saturation at maximum limits
- Positive feedback loop from perspective effects

### Solution: PID Controller with Filtering

Implemented a full PID controller with:

**1. PID Gains (Conservative, Stable)**
```python
pitch_kp = 0.08    # Proportional: down from 0.3
pitch_ki = 0.005   # Integral: small for steady-state
pitch_kd = 0.15    # Derivative: ~2x Kp for damping

roll_kp = 0.08
roll_ki = 0.005
roll_kd = 0.15
```

**2. Control Limits**
```python
max_pitch = 150    # 15% authority (down from 30%)
max_roll = 150
```

**3. Filtering & Smoothing**
```python
alpha = 0.4                          # Exponential filter
x_threshold = 30                     # Deadzone (up from 20)
y_threshold = 30
max_control_change_per_update = 50  # Rate limiting
```

### How It Works

1. **Detection**: Camera captures frame → LaserTracker detects green laser point
2. **Error Calculation**: Calculate pixel offset from frame center (dx, dy)
3. **Filtering**: Apply exponential moving average to smooth noise
4. **Deadzone**: Ignore errors within threshold (±30 pixels)
5. **PID Calculation**:
   - P term: Proportional to error
   - I term: Accumulated error over time (with anti-windup)
   - D term: Rate of change of error (provides damping)
6. **Rate Limiting**: Constrain control changes to prevent jerky motion
7. **Output**: Send roll/pitch commands via MAVLink at 20 Hz

### Expected Performance

| Metric | Target |
|--------|--------|
| Rise time | 1-2 seconds to reach 90% |
| Overshoot | <5% |
| Settling time | <3 seconds within ±10 pixels |
| Steady-state error | <10 pixels |
| Motion quality | Smooth, no oscillations |

## Tuning Guidelines

### When to Tune

Start with default values. Only tune if you observe specific issues:

### Tuning Parameters

**If response is too slow/sluggish:**
```python
pitch_kp = 0.10  # Increase by 20% from 0.08
roll_kp = 0.10
```

**If oscillations occur:**
```python
pitch_kd = 0.22  # Increase by 50% from 0.15
roll_kd = 0.22
```

**If steady-state error remains (offset persists):**
```python
pitch_ki = 0.01   # Increase from 0.005
roll_ki = 0.01
```

**If motion is jerky/noisy:**
```python
alpha = 0.3  # Decrease for more smoothing (from 0.4)
```

**If response is delayed:**
```python
alpha = 0.5  # Increase for less lag (from 0.4)
```

### Ziegler-Nichols Method (Advanced)

For optimal tuning:

1. Set Ki=0, Kd=0
2. Increase Kp until sustained oscillations occur (note this as Ku)
3. Measure oscillation period Tu (seconds)
4. Calculate:
   - Kp = 0.6 × Ku
   - Ki = 2 × Kp / Tu
   - Kd = Kp × Tu / 8

## Key Files Reference

| File | Purpose | Location |
|------|---------|----------|
| `ardupilot_quadrotor.py` | Main application, control loop | `client/python/example_user_scripts/ardupilot/` |
| `laser_steering.py` | PID controller implementation | `client/python/example_user_scripts/ardupilot/` |
| `laser_tracker.py` | Green laser detection | `client/python/example_user_scripts/ardupilot/` |
| `ardupilot_controls.py` | MAVLink & keyboard interface | `client/python/example_user_scripts/ardupilot/` |
| `scene_ardu_quadrotor.jsonc` | Scene configuration | `client/python/example_user_scripts/ardupilot/sim_config/` |
| `robot_ardu_quadrotor.jsonc` | Drone configuration | `client/python/example_user_scripts/ardupilot/sim_config/` |
| `BlocksMap.umap` | Unreal environment | `unreal/Blocks/Content/` |

## Keyboard Controls

| Key | Action |
|-----|--------|
| **W/S** | Pitch forward/backward |
| **A/D** | Roll left/right |
| **Q/E** | Yaw left/right |
| **↑/↓** | Increase/decrease throttle |
| **F** | Toggle laser steering ON/OFF |
| **T** | Arm/disarm motors |
| **L** | Land mode |
| **M** | Toggle STABILIZE/ALT_HOLD |
| **G** | Reset drone to starting position |
| **Space** | Reset to level (roll=0, pitch=0) |
| **0-9** | Set thrust 0.0-0.9 |
| **R** | Reset all controls |
| **H** | Show help |
| **ESC** | Quit |

## Common Issues & Solutions

### Issue: Drone still oscillates with PID

**Solution:** Increase derivative gain (Kd) by 50%
```python
pitch_kd = 0.22  # from 0.15
```

### Issue: Drone doesn't reach laser (steady-state error)

**Solution:** Increase integral gain (Ki)
```python
pitch_ki = 0.01  # from 0.005
```

### Issue: Response is too aggressive/fast

**Solution:** Reduce proportional gain (Kp)
```python
pitch_kp = 0.06  # from 0.08
```

### Issue: Control is jittery/noisy

**Solution:**
1. Increase filtering (lower alpha): `alpha = 0.3`
2. Increase deadzones: `y_threshold = 40`
3. Reduce rate limiting: `max_control_change_per_update = 30`

### Issue: Laser detection fails

**Check:**
1. Lighting conditions - laser must be bright
2. HSV color range in `laser_tracker.py` (lines 23-26)
3. Camera exposure settings
4. Minimum radius threshold (default 1 pixel)

### Issue: ArduPilot SITL not connecting

**Solution:**
1. Ensure SITL is running on UDP port 14550
2. Check connection string: `udp:127.0.0.1:14550`
3. Verify AirSim-ArduPilot UDP ports: 9002/9003

## Development Workflow

### Testing New PID Parameters

1. Edit gains in [laser_steering.py](client/python/example_user_scripts/ardupilot/laser_steering.py) (lines 126-133)
2. Start simulation and arm drone
3. Press 'F' to activate laser steering
4. Monitor logs for:
   - `dx_raw` vs `dx_filt` (filtering effectiveness)
   - `roll`, `pitch` values (should stay < 150)
   - Error convergence (dy should decrease steadily)
5. Observe drone behavior:
   - Should approach laser smoothly
   - Should settle with <10 pixel error
   - No oscillations or overshooting

### Log Format

```
Laser PID - dx_raw: 34.0, dy_raw: 18.0, dx_filt: 30.5, dy_filt: 16.2, roll: 45, pitch: 24
```

- `dx_raw/dy_raw`: Raw pixel error from frame center
- `dx_filt/dy_filt`: Filtered error after exponential smoothing
- `roll/pitch`: Final control outputs (-1000 to +1000 range)

## Technical Details

### Control Flow

```
FPV Camera (20 FPS)
    ↓
LaserTracker.process_frame()
    ↓ (dx_raw, dy_raw)
ExponentialFilter.update()
    ↓ (dx, dy)
Apply deadzone threshold
    ↓ (dx_error, dy_error)
PIDController.update()
    ↓ (P + I + D terms)
Rate limiting
    ↓ (roll, pitch)
ArduPilot.set_manual_control()
    ↓
ArduPilot SITL (physics)
    ↓
Drone moves in AirSim
```

### Coordinate Systems

- **Image coordinates**: Origin at top-left, Y increases downward
- **Laser below center**: dy > 0 → pitch forward (positive)
- **Laser right of center**: dx > 0 → roll right (positive)

### MAVLink MANUAL_CONTROL Range

```
roll:     -1000 (left)    to +1000 (right)
pitch:    -1000 (backward) to +1000 (forward)
throttle: 0 (down)        to +1000 (up)
yaw:      -1000 (CCW)     to +1000 (CW)
```

## Future Enhancements (Optional)

1. **Adaptive Gains**: Scale gains based on error magnitude
2. **Velocity Feedback**: Use optical flow or frame differencing
3. **Multi-Laser Support**: Track multiple laser points
4. **Distance Estimation**: Maintain fixed distance from target
5. **Yaw Control**: Rotate to keep laser centered using yaw
6. **Feed-forward Control**: Predict laser motion for moving targets
7. **Kalman Filtering**: More sophisticated state estimation

## Dependencies

```bash
pip install pymavlink pynput opencv-python imutils commentjson numpy
```

## Git Branch Structure

- `main`: Stable releases
- `main-fpv`: FPV camera and laser tracking development (current)

## Recent Commits

- `d117930`: Toggle laser steering and terminal suppress
- `ed70753`: Laser tracker rename
- `5918d71`: Added ball tracker
- `e12c7c6`: All manual controls work, removed to separate file
- `085880c`: Manual controls working

## Notes for Future Development

### Control Architecture Decisions

- **Why PID over MPC/LQR?** PID is simpler, more transparent, and sufficient for this single-loop vision servo task
- **Why 20 Hz control loop?** Matches camera frame rate and provides good responsiveness
- **Why exponential filter over Kalman?** Simpler, lower computational cost, adequate for this noise profile
- **Why rate limiting?** Prevents sudden control changes that ArduPilot flight controller may not handle smoothly

### Parameter Tuning Philosophy

- Start conservative (low gains) to ensure stability
- Tune for stability first, performance second
- Keep deadzones large enough to prevent micro-corrections
- Maintain control authority well below saturation limits
- Filter aggressively if in doubt - stability > speed

### Known Limitations

1. **No altitude control**: Throttle remains manual
2. **No yaw compensation**: Drone doesn't rotate toward laser
3. **Single laser only**: Cannot track multiple targets
4. **No distance estimation**: Approaches until laser is centered
5. **Perspective effects**: Close-range tracking may be less accurate

## Support & Resources

- Project AirSim Documentation: Check project repository
- ArduPilot SITL Documentation: https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html
- PID Tuning Guide: https://en.wikipedia.org/wiki/PID_controller#Manual_tuning

---

**Last Updated:** February 8, 2026
**Primary Developer:** Nik
**AI Assistant:** Claude Sonnet 4.5
