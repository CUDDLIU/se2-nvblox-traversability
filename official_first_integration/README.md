# FIRST local planning integration

Independent C++ integration of the M20 official FIRST trajectory library with the existing ASA global planner. **Current status: supervised hardware trials; continuous navigation and stairs are not accepted yet.** ScanPlanner's optimizer is not loaded by this entry point.

```bash
bash /home/nvidia/scanplanner_test/se2_nvblox_surface_shadow/official_first_integration/start_live.sh
# Explicit supervised execution; a new RViz goal is required after every start.
bash /home/nvidia/scanplanner_test/se2_nvblox_surface_shadow/official_first_integration/start_live.sh --execute --enter-navigation
bash /home/nvidia/scanplanner_test/se2_nvblox_surface_shadow/official_first_integration/stop.sh
```

The default map is explicitly pinned to `scanplanner_integration/maps/20261012_confirmed_scene`. Use `--map DIRECTORY` to select another built map. `--initial-map-pose X Y Z YAW` supplies a measured localization search seed; it never bypasses scan matching. RViz displays the classified local obstacle points, measured M20 model and selected trajectory, plus ASA's global map and path. The default mode receives robot telemetry but sends no velocity, mode or gait changes. `--execute` enables the guarded sender; `--enter-navigation` explicitly requests navigation mode. The deployment uses a hash-verified same-recording SuperLIO reference for localization when `maps/localization_reference.json` matches the selected Mesh. Initial search covers ±2 m seeds with full yaw search, stays within 3 m horizontally and 0.5 m vertically of the search anchor, and excludes quality-failing candidates before ambiguity checks. Startup defaults to searching near the recording start; the last matched pose is retained only as diagnostic output and is never automatically used as the next search center. `--recording-start` remains a compatible explicit spelling of this default. Use `--initial-map-pose X Y Z YAW` to deliberately choose a different search center. All starts still require fresh matching and stability confirmation. This remains local relocalization, not arbitrary whole-building place recognition. Use RViz Publish Point to select the goal after localization succeeds. `--duration SECONDS` bounds the session lifetime. Executing starts without a route; old retained paths and commands are isolated through unique session topics.

Stopping terminates the sender before the GUI/planners, sends three zero-velocity packets 33 ms apart, and requests restoration of the previous usage mode if this session changed it. `runtime/SESSION/telemetry/shutdown.json` records these sends; UDP delivery is not guaranteed by a successful send call. The session cleans up only processes it owns.

## Implemented

- All 4,641 official constant body-velocity primitives and four correspondence tables, with strict parsing and captured hashes. `evidence/audit.md` distinguishes verified binary behavior from unfinished state-machine work.
- Hard collision occurrence counts; binary soft-layer flags; audited goal/yaw/lateral/proximity score and ordered candidate validation. Native candidates are selected under speed limits; there is no output multiplier or hand-written recovery motion.
- A bounded continuity preference in native trajectory ranking reduces near-tie switching, using only the most recently published valid follow command from the same safety generation (under 120 ms old). It adds at most 0.08 to the native score, with x/y/yaw delta weights 0.10/0.16/0.16. Candidates still pass the official tables, current limits and full collision checks. Holds invalidate the preference immediately; there is no low-pass filter after validation and no delayed zero command. This is a project extension, not a recovered official binary score term.
- Audited native terminal controller: approach inside 0.5 m, position tolerance 0.1 m, then yaw alignment. Its actual command is independently swept and checked; route arc/floor guards prevent early arrival on overlapping routes.
- New obstacle scans are accepted in stable standing (gait 0x1001, no pending gait change) or a confirmed navigation gait after reported height remains stable for at least 0.3 s (within 1.5 cm). Queued scans acquired before stabilization are discarded; existing occupancy is retained. Stable standing populates the obstacle preview before any goal is selected; other unsupported postures do not. Motion still requires a route and confirmed navigation gait. MotionStatus source timestamps prevent callback delays from refreshing stale height.
- M20/AIRY near-body filtering rejects raw rays that cross the strict interior of the central chassis bottom face, using the URDF box and decoder optical-origin offsets. Endpoints remain unchanged; rejected returns create neither hits nor clearing rays. This targets the captured low-channel cluster without a blanket minimum-range increase. The collision geometry is an approximation of the opaque housing; this is a limited geometric hypothesis, not a calibrated multipath detector. Close-object fixtures pass; physical acceptance is recorded in the execution evidence.
- Before voxel deduplication, deskewed returns within 1.5 m of their physical sensor require two distinct neighboring points within 4 cm. Distant sparse returns bypass this filter. Rejected returns create neither occupied cells nor clearing rays. Thin-object fixtures pass, but single-return real objects can be suppressed; hardware acceptance remains required. This uses the [radius-neighbor filtering principle](https://pointclouds.org/documentation/tutorials/remove_outliers.html).
- The RViz odometry arrow is replaced by the M20 URDF in an isolated `first_robot/*` TF tree. Map-aligned odometry drives the body pose; telemetry drives calibrated leg joints. Wheel rotation is a relative integral of measured wheel speed, not an absolute measured angle. These visualization nodes send no robot commands.
- RViz shows `/first/planning_obstacles` as ordinary points using the same ground/height classification as local collision checks. Raw occupancy remains an internal diagnostic topic and is no longer displayed.
- Independent per-return LiDAR deskew using measured odometry, front/rear physical ray origins, occupied/free/unknown voxels, direct-ray clearing, and hit precedence. Silence does not clear obstacles.
- Saved surface heights are optional references for obstacle/ground separation and trajectory height, not support certificates. Missing center or footprint support never rejects a local candidate. Body odometry minus reported body height supplies the current ground datum. Saved heights are shifted to that datum and contribute only relative relief; they never replace the measured ground height. Where a reference is absent, candidate height continues from the previous sample. Real-time occupied-voxel collision checks retain the physical footprint. This does not implement live hole/drop-off detection.
- ASA route progress follows arc order with a 2.5 m target and corner preservation. A separate route revision resets progress only on route replacement/cancellation; localization/posture changes still invalidate commands and cloud evidence while retaining progress in the fixed map. Floor height gates prevent jumping to the nearest XY route on another floor.
- Stair gait request ahead of a height transition (a five-sample, 5 cm metric median is used only for gait classification to reject isolated mesh-height spikes); no routine pause while waiting on flat terrain. A single outstanding gait request, RL-state restriction and telemetry confirmation are implemented. Unconfirmed gait blocks entry. Full-body clearance from height transitions controls return to flat gait.
- Perception reuses epoch-tagged flat key tables for neighbor bins and per-sensor voxel deduplication. FIFO neighbor order, distinct-point tests and first-return selection are unchanged; rejected returns still generate no clearing rays.
- Exact collision checks use a spatial broad phase, static triangle heights at voxel centers are cached without caching live occupancy or floor choice, and an unchanged selected primitive is validated once. Per-return acquisition logging is opt-in with `--trace-near-body`; timing and rejection counters remain available without it.
- Separate perception, planning and 30 Hz command publisher threads; stale route, localization, cloud, pose or robot state suppress commands. Existing UDP command-age and robot-status watchdogs are retained.
- Explicit session ownership lock shared with the legacy launcher; only owned subprocesses stop. Sources, assets, builds, map derivatives and runtime records stay inside the original project.

Hardware telemetry defaults to `status_relay.py`: `AOS 10.21.33.103:30000 → NOS 10.21.33.106 → SSH → Jetson`. The transient NOS process only sends heartbeat 100/100, writes no files, and exits with the owned SSH connection. The original UDP socket still sends control commands and receives their acknowledgements/errors; direct UDP status cannot override the relay’s state. No official robot service or system network configuration is changed.

The relay retains NOS kernel receive timestamps. Expired, future-dated and reordered packets do not refresh watchdogs; disconnect immediately invalidates state and retries automatically without blocking the sender. Jetson and NOS clocks must remain synchronized. Authentication uses existing SSH keys, `FIRST_NOS_SSH_PASSWORD`, or the ignored mode-600 file `runtime/private/nos_ssh_password`. The deployed machine is configured; credentials are not part of the repository. `robot.py --status-source udp` is available for diagnostics, and localhost tests use UDP by default.

The external AOS-to-Jetson return path still loses packets: a simultaneous trace saw 395 outgoing MotionStatus frames but only 259 received. This fix routes this project’s telemetry around that path; it does not claim to repair the USB adapter/cable/switch or to fix UDP command acknowledgements. See `evidence/telemetry-fix.md`.

Robot status is still required: odometry provides body pose, but does not carry clearance above the ground, confirmed gait, navigation/manual mode or emergency-stop state. BasicStatus and MotionStatus freshness gates are unchanged by the removal of saved-map support checks. ASA’s existing global map and start checks are also unchanged.

Configured native candidate limits: normal vx ≤ 1.5 m/s, |vy| ≤ .4 m/s, |yaw| ≤ .8 rad/s, planar speed ≤ 1.5 m/s; stairs .4/.3/.4, planar .4 m/s. All native candidate increments remain available under these caps, including gentle turns. Terminal position/yaw minimum handling follows the audited native controller; no post-planner multiplier is used. Robot deadband calibration remains unverified.

## Build and tests

```bash
bash official_first_integration/build.sh
python3 -m unittest discover -s official_first_integration/tests -p 'test_*.py'
# ROS synthetic tests require an isolated domain; they never start a robot sender.
ROS_DOMAIN_ID=86 ROS_LOCALHOST_ONLY=1 python3 official_first_integration/tests/ros_chain.py
ROS_DOMAIN_ID=86 ROS_LOCALHOST_ONLY=1 python3 official_first_integration/tests/ros_chain.py --stairs
ROS_DOMAIN_ID=86 ROS_LOCALHOST_ONLY=1 python3 official_first_integration/tests/ros_chain.py --no-support
ROS_DOMAIN_ID=86 ROS_LOCALHOST_ONLY=1 python3 official_first_integration/tests/udp_loopback.py --gait --normal-speed
ROS_DOMAIN_ID=86 ROS_LOCALHOST_ONLY=1 python3 official_first_integration/tests/udp_loopback.py --gait-timeout --normal-speed
ROS_DOMAIN_ID=86 ROS_LOCALHOST_ONLY=1 python3 official_first_integration/tests/ros_chain.py --no-support --session
ROS_DOMAIN_ID=86 ROS_LOCALHOST_ONLY=1 python3 official_first_integration/tests/ros_chain.py --height-offset --session
ROS_DOMAIN_ID=86 ROS_LOCALHOST_ONLY=1 python3 official_first_integration/tests/ros_chain.py --posture --session
ROS_DOMAIN_ID=86 ROS_LOCALHOST_ONLY=1 python3 official_first_integration/tests/ros_chain.py --route-progress --session
ROS_DOMAIN_ID=86 ROS_LOCALHOST_ONLY=1 python3 official_first_integration/tests/udp_loopback.py --gait --normal-speed --enter-navigation --session
```

Use the project's ROS-enabled Python environment on Jetson. Vendor assets are not included in Git. `import_assets.py official_planner.tgz` verifies the captured archive against `evidence/manifest.json`. Core tests also build on a workstation with `cmake -S official_first_integration -B official_first_integration/build`, without any ScanPlanner or ROS dependency.

## Acceptance still required

1. Compare the recovered DWA score and terminal controller against the running original binary, and finish line-mode/state transitions. The entire original state machine is **not** behavior-equivalent or fully recovered yet.
2. Validate recorded stairs/open risers/landings and gait timing. Saved-map support rejection was removed at the user’s request; capture14 now produces a native candidate in the offline probe. This is not physical stair acceptance or a replacement live negative-obstacle detector.
3. Implement the real-time certified ASA start-connection when the saved navmesh rejects an otherwise valid measured start. The current global query still uses its existing start checker.
4. Complete supervised driving acceptance. Session `20261012_150133_1fcbf8c9` confirmed navigation mode, flat gait and 0.96 m measured displacement toward the existing ASA goal. The planner then rejected all speed-admissible candidates through the hard collision table. Stop packets and restoration of manual mode were exercised; the subsequent read-only capture confirmed mode 0. After correcting the saved-ground height offset, a further trial advanced 0.575 m but stopped again on accumulated low voxels; this remains partial execution acceptance. See [execution trials](evidence/execution-trials.md) for evidence and unresolved posture-transition behavior. The October 13 flat-route trial reached the original goal and the operator reported essentially smooth motion. Rare collision holds and cross-floor execution remain unaccepted; the subsequent continuity preference has separate synthetic validation and awaits field observation.
5. Measure timing under sustained navigation, including stairs. The first hardware trial averaged 30.003 Hz UDP sending, but the maximum send interval was 111.4 ms, so an uninterrupted 20 Hz minimum is not yet established. Motion/basic status receive gaps remained below 0.140/0.546 s; the seconds-long telemetry loss did not recur.

Legacy start scripts remain unchanged until these acceptance conditions pass. Do not interpret this directory as a completed ScanPlanner replacement.
