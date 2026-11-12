# CONTEXT — RedWvn/ardupilot (Redwing fork)

Read this first. Upstream ArduPilot documentation still applies to everything not listed here; this file says
what Redwing changed, how to build, test and release it, and why. Full detail is in the "Ardupilot development
KT" (KT Document Map, Drive KT folder).

## Branches

Each custom branch is the official upstream tag plus a few Redwing commits, so
`git log <tag>..<branch>` is the complete list of Redwing changes.

| Branch | Base | Contents | Status |
|---|---|---|---|
| `rw-plane-4.6.3-custom` | `Plane-4.6.3` | Multi-marker precision landing support: `PLND_TGT_ID` / `PLND_TGT_ID_BCKP` params and Lua `LANDING_TARGET` bindings | In service on the VTOL fleet |
| `rw-copter-4.6.3-custom` | `Copter-4.6.3` | Same changes ported to Copter | RQ-850 precision drop and copter SITL |
| `issue/2-qtakeoff-yaw-align-4.6.3` | `rw-plane-4.6.3-custom` | QTAKEOFF yaw align before transition (`Q_TKOFF_YAW_EN/RATE/DLY`), autotests, unit test | Flight testing, PR #3 not merged |
| `rw-plane-4.5.5-custom` | `Plane-4.5.5` | Same precision landing changes on 4.5.5 | Superseded |
| `radar_CAN_driver` | Plane-4.5 line | Benewake CAN driver with CAN ID filtering for three radars | Not ported to 4.6.3 |
| `rw_plane`, `rw_copter`, `add_custom_log` | 4.5 | 2024-2025 training branches | Stale, do not build from them |

Naming: `rw-<vehicle>-<version>-custom` for a release branch, `issue/<n>-<short-name>-<version>` for a feature
branch merged into it.

## Redwing changes on rw-plane-4.6.3-custom

| File | Change |
|---|---|
| `libraries/AC_PrecLand/AC_PrecLand.cpp` / `.h` | `PLND_TGT_ID` (index 19) and `PLND_TGT_ID_BCKP` (index 20), default -1 |
| `libraries/AP_Scripting/generator/description/bindings.desc` | `precland:handle_msg(mavlink_landing_target_t, time_ms)` and the `mavlink_landing_target_t` userdata |

The Lua companion script builds a `LANDING_TARGET` message from the JeVois serial data and feeds it to the
estimator through `handle_msg`. The flight scripts are in `redwing42/lua-scripts`, not here.

## Develop

| Item | Value |
|---|---|
| OS | Ubuntu 22.04 or 24.04 (native or WSL2); 20.04 is no longer supported by the install script |
| Toolchain | Installed by `Tools/environment_install/install-prereqs-ubuntu.sh -y` (`gcc-arm-none-eabi-10-2020-q4-major` in `/opt`, Python 3 venv with pymavlink, MAVProxy, empy) |
| PATH | The install script adds the toolchain and `Tools/autotest` to `~/.profile`; run `. ~/.profile` or log out and in once |
| Paths on the sim laptop | Plane tree `~/ardupilot` (`rw-plane-4.6.3-custom`), copter tree `~/ardupilot-copter` (`rw-copter-4.6.3-custom`) |

```bash
git clone --recurse-submodules https://github.com/RedWvn/ardupilot.git && cd ardupilot
git remote add upstream https://github.com/ArduPilot/ardupilot.git && git fetch upstream --tags
git checkout rw-plane-4.6.3-custom && git submodule update --init --recursive
Tools/environment_install/install-prereqs-ubuntu.sh -y && . ~/.profile
```

Run `git submodule update --init --recursive` after every switch between 4.5.5 and 4.6.3 branches; stale
submodules are the most common cause of odd build errors (`./waf submodulesync` or
`./waf submodule_force_clean` if stuck).

## Build

```bash
./waf configure --board CubeOrangePlus   # or CubeOrange, Pixhawk6X, Pixhawk6C, CUAV-X7, sitl
./waf plane                               # ./waf copter on the copter branch
ls build/CubeOrangePlus/bin/              # arduplane.apj (Mission Planner) and arduplane_with_bl.hex
```

`./waf distclean` after switching ArduPilot versions. Fleet boards are CubeOrange and CubeOrangePlus (H7).

## Test

1. SITL: `Tools/autotest/sim_vehicle.py -v ArduPlane -f quadplane --console --map`; put Lua scripts in a
   `scripts/` folder where SITL starts and set `SCR_ENABLE = 1`.
2. Autotests for a feature branch, e.g. QTAKEOFF: `Tools/autotest/autotest.py build.Plane test.QuadPlane`.
3. Camera-in-the-loop (precision landing / drop): `redwing42/redwing-sim-tool` (Gazebo + SITL).
4. Ground test, then one aircraft with a standby pilot from a written test plan, then the fleet.

## Release

1. PR the feature branch into the release branch; merge after test evidence.
2. Build every fleet board and rename `arduplane.apj` to `arduplane-<version>-custom-<board>.apj`.
3. Upload to the Drive Firmware folder (Build/Stable/<version>-Custom/<board>) only after HQ testing passes;
   update the Current aircraft configuration sheet at rollout.
4. Flash with Mission Planner (save params first, Install Firmware → Load custom firmware, check the version
   banner and git hash after reboot, reload and compare params). Do not downgrade this way.

Gap: builds are not tagged yet. Tag each build that goes on an aircraft (e.g. `rw-plane-4.6.3-r1`) and put the
tag in the file name so a binary can be traced to its commit.

## Code-level learnings

- `PLND_TGT_ID` / `PLND_TGT_ID_BCKP` are only declared in C++; `AC_PrecLand` does not filter by ID. Marker
  selection is done by the Lua scripts, so it is a Lua change, never a firmware change.
- ArduPlane does not run `AC_PrecLand_StateMachine`, so `PLND_RET_*` retry params do nothing on Plane; retries are
  in Lua.
- Never reuse or renumber a parameter index: values are stored by index, so a change silently resets or swaps
  params on aircraft. Check upstream master for the same group before picking an index, so a future release does
  not take it.
- New Lua bindings need a `depends` clause in `bindings.desc` so boards without the feature still build, and
  should also be added to `libraries/AP_Scripting/docs/docs.lua` (not done yet for `handle_msg`).
- A script that calls a Redwing binding fails at load on stock firmware with an unknown method error.
- QTAKEOFF params use `var_info2` indices 40 to 42, which upstream master already uses (TKOFF_RPM_MIN/MAX,
  THRST_LOSS_OPT). Move them to free indices on the next upgrade and tell ops the values will reset.
- 4.5.5 to 4.6.3: GPS params were renamed (GPS_xxx to GPS1_/GPS2_) and FS_SHORT_TIMEOUT was removed; re-check
  both after loading an old param file.

## Key decisions (why)

| Decision | Reason |
|---|---|
| Fork with thin branches on official tags | The diff against the tag is the full list of Redwing changes, which makes porting to a new release a cherry-pick. |
| Minimal C++ change (params + one binding), logic in Lua | Scripts can be updated by MAVFtp without reflashing; firmware changes are rare and need a full test cycle. |
| Move the fleet from 4.5.5 to 4.6.3 (Jun 2026) | 4.6.2 TECS fix for the premature de-transition in tailwind (10 May 2026), H7 flash/ECC fixes, better QASSIST/VTOL recovery; see "Firmware upgrade and changelog 4.5.5 -> 4.6.3". |
| QTAKEOFF yaw align as a firmware change | It changes the QuadPlane takeoff controller (`quadplane.cpp`, `verify_vtol_takeoff`) before the transition starts. |
