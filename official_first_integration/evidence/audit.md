# Official FIRST implementation audit

Source: NOS `/opt/robot/share/planner`, ELF build ID `c99ebf84f89e85be231a2b63da1beaeb9d941c1d`. Exact files and hashes: `manifest.json`. Binary and tables are local vendor assets, excluded from Git; no license or source availability is inferred.

Verified from assets and AArch64 disassembly:

- 4,641 primitives, ten samples at 0.1 s, one second horizon. Samples match forward Euler integration of stored body velocity, not an analytic arc. Test checks every sample (20 μm tolerance for text rounding).
- Hard collisions: `updatePathPenalty` 0xb9658–0xb966c increments for every table occurrence. Repeated IDs within a row matter.
- Soft flags: 0xb9688–0xb9700 sets each path flag to one. It does not sum hits or divide by sample count. The historical Python planner differs here.
- Score: endpoint Euclidean distance, wrapped heading error towards the local goal, absolute lateral velocity, three binary proximity flags, and a +1 penalty when the goal is in front and native vx ≤ 0.05 (`0xd4288`). See 0xb9f90–0xba0cc.
- YAML parameter offsets 272/280/288 are goal/yaw/lateral. Offsets 296/304/312 are weight_ob1/2/3 (`loadParameters` 0x974cc–0x975e0). The binary multiplies soft arrays at offsets 264/288/312 by parameters 312/304/296 respectively: innermost soft table is weighted 1.0, middle .8, outer .6. This is opposite to the table labels' apparent numeric order.
- Loader: PathData offsets 72/96/120/144 correspond to base, _4, _5, _6. `readCorrespondences` 0xc1a7c–0xc1c78.
- `PlannerLoop` chooses separate `runDWA` / `runLine`. `runDWA` checks cancellation, initializes zero velocity, snapshots grid, evaluates penalties/candidates, validates selected path, computes command and publishes. Candidate rejection can retry up to 100 times.

- `CmdCalculate` 0xbd830–0xbd920: outside 0.5 m it copies native selected velocity; inside, until 0.1 m, it commands goal error with the native proportional/minimum-component loop (start multiplier 0.5, increment 0.05; x/y threshold 0.25).
- `CmdCalculate` 0xbdd74–0xbe064: inside position tolerance, zero linear velocity, wrap yaw error, accept within 0.1 rad; otherwise use signed 0.7 rad/s for smaller errors or 1.5 × error, then validate rotation. Project gait caps limit the command before its actual swept collision check.
- `cmdVelLimit` 0xba9a8–0xbad64 clamps maximum x/y/yaw and processes cancellation/status. No per-axis minimum candidate filter was found here. Extra minimum-component candidate filters were removed to retain native gentle curves.

Project extension after the audited score: an optional previous-command continuity cost, capped at 0.08, is added during native candidate ranking. It uses only a fresh valid same-generation command and never changes an output velocity after collision validation. The `score()` kernel above remains unchanged; the augmented ranking is deliberately not claimed to reproduce the official binary.

Not yet established as behavior-equivalent:

- Full outer `CmdCalculate` status handling, `LinePlanner` turn/approach/stop transitions, acceleration handling and all numerical planner status meanings.
- Full original-binary input/output comparison. The binary uses DrDDS and is not a ROS executable; it has not been executed against synthetic input.
- New stair gait logic and 3D terrain adaptation are project extensions, not recovered official source.

The independent ROS node currently implements DWA selection, the audited terminal controller and fail-closed input/cancel handling. It must not be described as a complete recovered official state machine. Hardware output is opt-in with `--execute`; supervised flat-route trials are documented separately. These trials do not establish complete state-machine equivalence or staircase acceptance.
