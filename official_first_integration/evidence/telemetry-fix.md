# M20 status reception: external return-path loss

Date: 2026-10-12. No velocity, gait or navigation-mode commands were sent to the physical robot during these diagnostics.

## Evidence

- Baseline kernel timestamps on Jetson: MotionStatus gaps up to 2.309 s and BasicStatus gaps up to 6.527 s. Application receive delay stayed under 34 ms.
- Direct TCP did not remove delays: it recovered source frames generated 100 ms apart but delivered them in bursts after delays of 1.7–3.2 s. It was not selected as the fix.
- Explicit UDP status queries 1002/4 and 1002/6 returned error 57347 on this firmware and did not help. The deployed receiver does not send these unsupported queries.
- AOS `basic_server` UDP port 30000 had zero socket drops; unrelated system UDP drop counters were not evidence against this socket. USB autosuspend was already disabled on the parent device (power/control `on`, runtime `active`). No buffer or power settings were changed.
- Simultaneous AOS eth2 capture and Jetson application log: AOS sent 395 complete MotionStatus datagrams, Jetson received 259; source maximum spacing was 109 ms, received maximum spacing 3.315 s. All 74 heartbeats within the comparison window reached AOS. Capture kernel drops were zero. This locates the observed failure on the external AOS-to-Jetson return path, after packet generation; it does not isolate the precise USB adapter/cable/switch failure.
- AOS eth2 is a CDC Ethernet USB 10/100 LAN device, vendor/product 0fe6:9900. A six-second sample measured about 46 Mbps RX. Link counters alone did not establish the physical cause.
- A 40-second probe from NOS through its internal eth0 network to AOS 10.21.33.103 received 399 MotionStatus frames (max spacing 102.8 ms) and 79 BasicStatus frames (max spacing 503.7 ms).

Raw records are under `official_first_integration/runtime/transport_baseline`, `transport_explicit_subscriptions`, `transport_both_ends`, and `telemetry_nos_fixed` on Jetson. Packet captures and runtime credentials are excluded from Git.

## Implemented change

Only this project's state reception changes: AOS internal UDP → NOS → authenticated SSH stream → Jetson. `status_relay.py` runs a transient, heartbeat-only Python worker on NOS. No NOS files or robot services are installed/modified. The normal launcher automatically uses the new receiver; its original UDP control socket and native command values remain unchanged. UDP replies to control requests still feed command-error handling, while direct UDP state cannot overwrite authoritative relayed state.

NOS kernel receive timestamps are preserved across the stream. Buffered old data retains its original age. Parsing rejects stale/future/out-of-order messages; a lost connection invalidates both status ages and retries. Connection retirement does not block the sender loop. Existing 350 ms motion, 1.2 s basic, and 120 ms command watchdogs are unchanged. Synchronized NOS/Jetson clocks are required; a clock fault fails closed.

Authentication uses a key or an environment/private runtime credential. No password is stored in tracked files. The private deployment file is mode 600 inside an ignored mode-700 runtime directory.

## Validation and limits

- Eight gate/relay tests cover source age, fragmented/coalesced stream frames, stale/future/reordered data, heartbeat-only worker behavior and disconnection. The worker test also exercises automatic reconnect and requires newly received state.
- Existing localhost UDP execution simulation passed at 30.006 Hz with native [1.2, 0, 0.8] commands, gait confirmation, watchdog stop and mode restoration.
- 124.17 s hardware health log: 1,230 MotionStatus and 246 BasicStatus frames. Maximum application receive gaps 133.46 ms and 534.07 ms; maximum frame age 37.06 ms. One SSH connection, no rejected old/future/reordered/invalid frames, no stale-state health samples after startup, zero physical velocity packets.
- The observation establishes that the seconds-long status gaps were absent during this run. It is not a physical navigation acceptance test or proof that the external return path is repaired. Direct UDP acknowledgements can still be lost on that path.
