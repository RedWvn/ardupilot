# QTAKEOFF yaw alignment (Redwing)

Ported to `rw-plane-4.6.3-custom` (branch `issue/2-qtakeoff-yaw-align-4.6.3`).

## Objective

After a `NAV_VTOL_TAKEOFF` reaches its target altitude in AUTO, hold position and yaw toward the **next mission waypoint bearing** before calling the normal VTOL→fixed-wing transition. Climb, position hold, and transition logic are otherwise unchanged ArduPlane behavior.

## Mission flow

1. `NAV_VTOL_TAKEOFF` — `takeoff_controller()` climbs to `plane.next_WP_loc.alt` (stock).
2. `verify_vtol_takeoff()` — when `current_loc.alt >= next_WP_loc.alt`, start yaw align (if enabled).
3. `takeoff_controller()` — position-holds takeoff altitude; yaw slews to target at `Q_TKOFF_YAW_RATE` (capped).
4. `verify_vtol_takeoff()` — when heading error ≤ 5°, `Q_TKOFF_YAW_DLY` has elapsed, **and** yaw rate is low enough, returns true.
5. Stock path — `transition->restart()`, TECS reset, mission continues.

## Code touch points

| Location | Role |
|----------|------|
| `do_vtol_takeoff()` | Clears `tkoff_yaw_*` state on takeoff start |
| `takeoff_controller()` | Rate-capped angle yaw + Z position hold while aligning |
| `verify_vtol_takeoff()` | Gates `transition->restart()` until heading + rate settled |

Yaw uses `attitude_control->input_euler_angle_roll_pitch_yaw(..., slew_yaw=true)` with a temporary `Q_A_RATE_Y_MAX` cap from `Q_TKOFF_YAW_RATE`.

## Parameters

| Param | Default | Description |
|-------|---------|-------------|
| `Q_TKOFF_YAW_EN` | 1 | 0 = disable (transition immediately at altitude). 1 = yaw align before transition. |
| `Q_TKOFF_YAW_RATE` | 20 deg/s | Max yaw rate during align only. `0` = use normal ATC limits. Lower if overshoot/oscillation in wind. |
| `Q_TKOFF_YAW_DLY` | 1.5 s | Hold after heading within 5° before transition. Avoid 0 in wind. |

Heading tolerance is **5°** (hardcoded). Transition also waits until yaw rate ≤ `max(8, 0.4 * Q_TKOFF_YAW_RATE)` deg/s.

Yaw align runs only in **AUTO** with healthy AHRS and a valid **next nav** command after the takeoff item. Otherwise alignment is skipped and transition proceeds at altitude.

On 4.6.3 these live in `var_info2` at indices **40** / **41** / **42** (`APPROACH_DIST` occupies index 39).

## Flight-test findings (May 2026, Fighter-D)

Observed with the earlier “use stock ATC yaw” build (`Q_A_SLEW_YAW≈60°/s`, `Q_A_RATE_Y_MAX=45`, short/zero `Q_TKOFF_YAW_DLY`):

| Symptom | Cause | Mitigation in this branch |
|---------|-------|---------------------------|
| Transition at wrong heading / nose still swinging | Heading briefly entered ±5° while RATE.Y was 40–60+°/s; short DLY allowed transition mid-overshoot | Rate gate + restore `Q_TKOFF_YAW_RATE` (default 20) |
| Oscillatory yaw (settle MSG repeats) | Overshoot through target → leave 5° window → timer reset | Rate cap during align |
| Climb past takeoff alt during yaw | `set_climb_rate_cms(0)` after fast climb does not position-hold | Z `input_pos_vel_accel_z` at takeoff alt |
| High-wind difficulty holding heading for settle | Partly airframe/tune + gusts; partly too-aggressive yaw | Tune `Q_TKOFF_YAW_RATE` 15–25 and keep `Q_TKOFF_YAW_DLY` ≥ 0.5–1.5 |

`Q_A_RAT_YAW_*` / weathervane / weight still matter (tune), but the premature-transition and altitude-hold cases were **feature** gaps, not tune-only.

## Field testing guide

### Pre-flight setup

1. Confirm firmware build includes `Q_TKOFF_YAW_EN`, `Q_TKOFF_YAW_RATE`, `Q_TKOFF_YAW_DLY`.
2. Load a short AUTO mission: `NAV_VTOL_TAKEOFF` → waypoint with clear bearing → land/RTL.
3. Recommended starting params:

| Param | Value | Notes |
|-------|-------|-------|
| `Q_TKOFF_YAW_EN` | 1 | Feature under test |
| `Q_TKOFF_YAW_RATE` | 20 | Start here; 15 if light airframe / gusty |
| `Q_TKOFF_YAW_DLY` | 1.0–1.5 | Do **not** use 0 for first field flights |
| `Q_WVANE_ENABLE` | 0 | Avoid fighting yaw align during bring-up |

4. Pilot on TX ready to take over (QHOVER/QLAND) once yaw is confirmed, before or during transition if needed.
5. Note wind direction/speed and aircraft heading on the ground relative to WP2 bearing (0/45/90/…/315° offsets).

### Per-run checklist

1. Arm in AUTO; aircraft climbs to takeoff altitude.
2. Confirm hover / position hold (altitude should **not** keep climbing).
3. Confirm yaw toward next WP; watch GCS for `Takeoff yaw aligned, settling Xs`.
4. At transition start, heading should be within **±5°** of WP bearing and yaw rate visibly settled (no rapid left/right swing).
5. Allow transition, or take manual control after yaw confirm to save time.
6. Log pass/fail, offset angle, wind, and param values used.

### Pass / fail criteria

| Check | Pass |
|-------|------|
| Altitude during align | Stays near takeoff alt (no continued climb of several metres) |
| Heading at transition | Within ±5° of next WP bearing |
| Settle behaviour | No endless oscillation; settle MSG then transition |
| Premature transition | Nose not still swinging through target when FW transition starts |

### Tuning if a case fails

| Symptom | Action |
|---------|--------|
| Overshoot / settle MSG repeats / transition heading off | Lower `Q_TKOFF_YAW_RATE` (e.g. 20 → 15) and/or raise `Q_TKOFF_YAW_DLY` |
| Yaw takes too long on large offsets | Raise `Q_TKOFF_YAW_RATE` toward 25–30 (re-check overshoot) |
| Still unstable at 180° in gusts | Check `Q_A_RAT_YAW_*` / `Q_A_SLEW_YAW` airframe tune; keep DLY ≥ 1.0 |
| Need baseline without feature | `Q_TKOFF_YAW_EN=0` |

### Suggested matrix

Repeat at ground heading offsets **0, 45, 90, 135, 180, 225, 270, 315°** relative to WP2 (same matrix as May 2026 Fighter-D tests). Prioritise 90° and 180° — those exposed premature transition and high yaw-rate instability.

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
- Recommended field start: `Q_TKOFF_YAW_RATE=20`, `Q_TKOFF_YAW_DLY=1.0` (or 1.5).

## Automated tests

### Math unit tests (yaw tolerance / wrap / rate gate)

```bash
cd ~/redwing/ardupilot
./waf configure --board sitl
./waf tests --targets tests/test_tkoff_yaw_align
./build/sitl/tests/test_tkoff_yaw_align
```

Covers heading tolerance, wrap-around, and the May 2026 premature-transition predicate (heading OK + high RATE.Y must **not** settle).

### QuadPlane SITL autotest

```bash
cd ~/redwing/ardupilot
./waf plane
./Tools/autotest/autotest.py --no-clean \
  test.QuadPlane.TakeoffYawAlignEnabled \
  test.QuadPlane.TakeoffYawAlignDisabled \
  test.QuadPlane.TakeoffYawAlignWest \
  test.QuadPlane.TakeoffYawAlignAlreadyNearTarget \
  test.QuadPlane.TakeoffYawAlignSettleDelay \
  test.QuadPlane.TakeoffYawAlignZeroDelay \
  test.QuadPlane.TakeoffYawAlignNoPrematureTransition \
  test.QuadPlane.TakeoffYawAlignNoNextNav \
  test.QuadPlane.TakeoffYawAlignAltitudeHold \
  test.QuadPlane.TakeoffYawAlignOpposite
```

| Test | Covers |
|------|--------|
| `TakeoffYawAlignEnabled` | EN=1, East WP, settle text, MC→FW transition |
| `TakeoffYawAlignDisabled` | EN=0, no settle text, fast transition |
| `TakeoffYawAlignWest` | ~270° target bearing |
| `TakeoffYawAlignAlreadyNearTarget` | Already within tolerance → settle-dominated timing |
| `TakeoffYawAlignSettleDelay` | `Q_TKOFF_YAW_DLY=3` honored after ≤5° |
| `TakeoffYawAlignZeroDelay` | `DLY=0` still requires ≤5° heading + low yaw rate |
| `TakeoffYawAlignNoPrematureTransition` | Flight regression: high ATC slew + short DLY; heading/rate settled at transition; no climb |
| `TakeoffYawAlignNoNextNav` | No next nav WP → skip yaw |
| `TakeoffYawAlignAltitudeHold` | Altitude held during settle (no continued climb) |
| `TakeoffYawAlignOpposite` | ~180° yaw; heading/rate settled at transition |

## Note on ArduCopter

Copter `NAV_VTOL_TAKEOFF` does not yaw to the next WP; this is ArduPlane-specific behavior gated in `verify_vtol_takeoff()`, similar in spirit to commanding a fixed heading before transition.
