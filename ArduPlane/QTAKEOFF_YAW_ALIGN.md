# Feature Profile: QTAKEOFF Yaw-Alignment Sub-State

## Objective
Implement a sub-state in the `QTAKEOFF` sequence that yaws the aircraft to face the heading of the next waypoint *before* initiating the fixed-wing transition.

## Why this is needed
For many hybrid VTOL aircraft (like the Redwing Labs models), transitioning while facing the direction of travel is more efficient and safer than transitioning from an arbitrary orientation. Currently, ArduPilot begins the transition immediately upon reaching the takeoff altitude (`Q_TAKEOFF_ALT`).

## Technical Overview
- **Trigger**: Reaching `Q_TAKEOFF_ALT` in an AUTO mission.
- **Action**: Aircraft holds position and yaws toward the bearing of the next mission waypoint.
- **Exit Condition**: Heading error is within a configurable threshold (`Q_TKOFF_YAW_THR`).
- **Safety**: Uses existing takeoff timeouts to prevent hanging if yaw alignment fails (e.g., due to motor issues or extreme wind).

## Proposed Changes
- **`ArduPlane/quadplane.h`**: Added parameter and state tracking variables.
- **`ArduPlane/quadplane.cpp`**: 
    - Registered `Q_TKOFF_YAW_THR`.
    - Modified `verify_vtol_takeoff` to gate the transition call.
    - Modified `takeoff_controller` to drive yaw toward the target bearing.

## Parameter Reference
- `Q_TKOFF_YAW_THR`: Heading error threshold in degrees. Default is 0 (disabled). Recommended value: 10-15 degrees.
