# QTAKEOFF yaw alignment (Redwing)

## Objective

After a `NAV_VTOL_TAKEOFF` reaches its target altitude in AUTO, hold position and yaw toward the **next mission waypoint bearing** before calling the normal VTOL→fixed-wing transition. Climb, position hold, and transition logic are otherwise unchanged ArduPlane behavior.

## Mission flow

1. `NAV_VTOL_TAKEOFF` — `takeoff_controller()` climbs to `plane.next_WP_loc.alt` (stock).
2. `verify_vtol_takeoff()` — when `current_loc.alt >= next_WP_loc.alt`, start yaw align (if enabled).
3. `takeoff_controller()` — `set_climb_rate_cms(0)` holds altitude; yaw slews to target via existing attitude API.
4. `verify_vtol_takeoff()` — when heading error ≤ 5° and `Q_TKOFF_YAW_DLY` has elapsed, returns true.
5. Stock path — `transition->restart()`, TECS reset, mission continues.

## Code touch points

| Location | Role |
|----------|------|
| `do_vtol_takeoff()` | Clears `tkoff_yaw_*` state on takeoff start |
| `takeoff_controller()` | Angle yaw + altitude hold while `tkoff_yaw_align_active` |
| `verify_vtol_takeoff()` | Gates `transition->restart()` until yaw aligned |

No new QuadPlane methods. Yaw uses `attitude_control->input_euler_angle_roll_pitch_yaw(..., slew_yaw=true)`, same pattern as `waypoint_controller()`.

## Parameters

| Param | Default | Description |
|-------|---------|-------------|
| `Q_TKOFF_YAW_EN` | 1 | 0 = disable (transition immediately at altitude). 1 = yaw align before transition. |
| `Q_TKOFF_YAW_DLY` | 1.5 s | Hold after heading within 5° before starting transition. |

Heading tolerance is **5°** (hardcoded in `verify_vtol_takeoff()`).

Yaw align runs only in **AUTO** with healthy AHRS and a valid **next nav** command after the takeoff item. Otherwise alignment is skipped and transition proceeds at altitude.

## Safety

Existing takeoff failure timeout (`takeoff_time_limit_ms`) still applies if climb or yaw align never completes.

## SITL

```bash
cd ~/redwing/ardupilot
sim_vehicle.py -v Plane -f quadplane --console --map \
  --add-param-file=../sitl/configs/vtol-medical.param
```

- `Q_TKOFF_YAW_EN=0` — baseline (no yaw hold before transition).
- `Q_TKOFF_YAW_EN=1` — expect hover at takeoff alt, yaw toward next WP, GCS “Takeoff yaw aligned, settling…”, then transition.

## Note on ArduCopter

Copter `NAV_VTOL_TAKEOFF` does not yaw to the next WP; this is ArduPlane-specific behavior gated in `verify_vtol_takeoff()`, similar in spirit to commanding a fixed heading before transition.
