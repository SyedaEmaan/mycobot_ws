# VERIFICATION.md — building trust in mycobot_csp

This package is a CSP (Cyclic Synchronous Position) sibling of the verified
`mycobot_csv` (CSV/velocity) baseline. CSP is the right mode for MoveIt
integration: the drive runs the position loop, so JTC just feeds it
interpolated targets.

Trust comes from three things, in order:

1. **CSV→CSP code-level diff is small and audited** (read by eye)
2. **Build cleanliness** (compiles without warnings)
3. **Runtime behavior on the bench** (move-and-hold test)

Do all three before trusting this plugin.

---

## Phase 1 — CSV→CSP diff checklist (15 minutes, no hardware)

This package was created by copying `mycobot_csv` and applying a tightly
scoped set of changes. Open `src/mycobot_csv/` and `src/mycobot_csp/`
side-by-side and confirm the following deltas — and **only** these
deltas — are present.

### Functional CSP diffs (six items)

- [ ] **Mode of operation byte (0x6060).**
      `mycobot_csv.cpp:configure_csv_mode()` writes `mode = 9`.
      `mycobot_csp.cpp:configure_csp_mode()` writes `mode = 8`. Readback
      verification updated to match.

- [ ] **RxPDO mapping entry 2 (0x1602:02).**
      CSV: `0x60FF0020` (Target Velocity).
      CSP: `0x607A0020` (Target Position).
      Comment in source updated.

- [ ] **RxPDO struct field name and semantics.**
      CSV: `int32_t target_velocity` (counts/sec).
      CSP: `int32_t target_position` (counts).
      Same offset (2), same width (4 bytes). Struct still 12 bytes.

- [ ] **Exported command interface.**
      CSV: `export_command_interfaces()` registers `HW_IF_VELOCITY`.
      CSP: registers `HW_IF_POSITION`.

- [ ] **on_init() validation.**
      CSV: requires `velocity` command interface in URDF.
      CSP: requires `position`. Error message updated.

- [ ] **write() unit conversion.**
      CSV: `cnt_per_sec = lround(cmd_rad_per_sec * counts_per_rad_)`.
      CSP: `cnt = lround(cmd_rad * counts_per_rad_)` — no division by dt.

### CSP-specific safety additions (NaN / startup)

These do not exist in the CSV plugin because they are not needed there:
under CSV, target = 0 means "stand still." Under CSP, target = 0 means
"sprint to encoder origin." Verify each is present.

- [ ] **Early roundtrip in `init_soem()`.**
      After `ecx_config_map_group()` and watchdog disable, before
      returning OK, `fieldbus_roundtrip()` is called once and
      `last_position_counts_[i] = txpdo_[s]->position_actual` for each
      joint.

- [ ] **`reach_operational()` priming.**
      Writes `rxpdo_[s]->target_position = last_position_counts_[i]`
      (NOT 0) into every joint before priming + OPERATIONAL request.

- [ ] **`enable_drive()` brake-release loop.**
      The 20-cycle brake-release loop writes
      `rxpdo_[s]->target_position = txpdo_[s]->position_actual` each
      cycle. Also: just before the Enable Operation transition,
      `target_position` is refreshed to the live `position_actual` so
      the drive enters OPERATION_ENABLED with a tight position window.

- [ ] **`disable_drive()`.**
      Both phases (brake-released wind-down and brake-engaged shutdown)
      write `rxpdo_[s]->target_position = txpdo_[s]->position_actual`,
      not 0.

- [ ] **`write()` NaN guard.**
      `if (!std::isfinite(cmd_rad)) cmd_rad = hw_positions_[i];`
      Falls back to current encoder position, not 0.

- [ ] **`on_activate()` post-enable seeding.**
      After all `enable_drive()` calls succeed, sets
      `hw_position_commands_[i] = hw_positions_[i]` so the first
      scheduler-driven `write()` already has a sane finite command.

### Renames (cosmetic but pervasive)

- [ ] Namespace `mycobot_csv` → `mycobot_csp`
- [ ] Class `MyCobotCSV` → `MyCobotCSP`
- [ ] PDO structs `csv_rxpdo_t`/`csv_txpdo_t` → `csp_rxpdo_t`/`csp_txpdo_t`
- [ ] Plugin string `mycobot_csv/MyCobotCSV` → `mycobot_csp/MyCobotCSP`
- [ ] Buffer `hw_velocity_commands_` → `hw_position_commands_`
- [ ] Helper names `slave_csv_config`/`configure_csv_mode` → `slave_csp_config`/`configure_csp_mode`
- [ ] CMake project, library, plugin XML library path, package.xml name

If you find any other functional difference beyond the eleven items above,
it is a bug. Flag and fix.

---

## Phase 2 — Build cleanliness (5 minutes, no hardware)

```bash
cd ~/mycobot_ws
colcon build --packages-select mycobot_csp --symlink-install
source install/setup.bash
```

Pass criteria:

- [ ] `Finished <<< mycobot_csp` with no `Failed` lines
- [ ] No `-Wpedantic` or `-Wextra` warnings about our code
      (warnings about SOEM headers are OK — those aren't ours)
- [ ] `ls install/mycobot_csp/lib/libmycobot_csp.so` exists
- [ ] Plugin description installed:
      ```
      cat install/mycobot_csp/share/ament_index/resource_index/hardware_interface__pluginlib__plugin/mycobot_csp
      ```
      should print `share/mycobot_csp/mycobot_csp_plugin.xml`.

If any fail: build issue, fix before Phase 3.

---

## Phase 3 — Runtime equivalence (15 minutes, hardware needed)

### 3.1 — Bus brings up

```bash
# Edit urdf/test_one_joint.urdf first: confirm <param name="ifname"> is
# correct (run `ip link` to check).

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch mycobot_csp verify.launch.py
```

Pass criteria — log should contain, in order:

- [ ] `EtherCAT bring-up: ec_init on '<X>'`
- [ ] `EtherCAT: 1 slave(s) discovered.`
- [ ] `Slave 1 enable entry: sw=0x0XXX (...)`
- [ ] `Slave 1: OPERATION_ENABLED, brake released. position_actual=<some int>`
- [ ] `MyCobotCSP ACTIVE — 1 drive(s) operating in CSP mode.`
- [ ] `[forward_position_controller] activate_successful` (after the 4 s delay)

### 3.2 — Drive HOLDS POSITION when no command published

This is the headline difference vs. the crude CSV-with-stop approach.
With the launch from 3.1 still running, in another terminal:

```bash
ros2 topic echo /joint_states
```

Watch for ~10 seconds without publishing any command:

- [ ] `position[0]` reads a stable value (encoder noise of a few counts is
      OK; sustained drift of >1 count/sec is NOT)
- [ ] `velocity[0]` reads ~0 (noise around zero is fine)

If the joint drifts here, the EtherCAT path is unreliable, the drive is
misbehaving, or counts_per_rad is wildly off — but the *plugin* logic
should be correct.

### 3.3 — Drive servos to commanded position and HOLDS

Note the current position p0 from `/joint_states`, then:

```bash
# Step +0.05 rad (about 2.9°)
ros2 topic pub --once /forward_position_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [<p0 + 0.05>]}"
```

- [ ] Joint moves smoothly to the new position
- [ ] Joint STOPS at the target without overshoot or oscillation
- [ ] Position is HELD there with no further command needed
- [ ] No fault: `following_error` (if you wire it into a relay) stays small

```bash
# Back to start
ros2 topic pub --once /forward_position_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [<p0>]}"
```

- [ ] Joint returns to p0 and holds

```bash
# Step -0.05 rad (other side)
ros2 topic pub --once /forward_position_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [<p0 - 0.05>]}"
```

- [ ] Joint moves the other direction and holds

### 3.4 — Cleanup

Ctrl-C the launch. Should see in the log:

- [ ] `Deactivating MyCobotCSP...`
- [ ] `Slave 1: disabled, brake engaged.`
- [ ] `EtherCAT socket closed.`

Re-launching after a clean shutdown should bring the bus back up without
a power-cycle.

---

## Outcome

When all three phases pass:

- The CSV→CSP migration is **verified by diff** — every change is
  documented and accounted for.
- The plugin **moves and holds** under autonomous topic commands, no
  MoveIt or RViz dependency.
- The plugin is ready for MoveIt integration via `csp_moveit.py` (Phase
  3 of the project plan).

If any phase fails: STOP, capture the log, ping the user. Do not work
around the failure — the whole point is a trusted artifact.
