# mycobot_csv + MoveIt 2 — Integration Notes

Working context for integrating the verified `mycobot_csv` ros2_control
plugin (CiA 402 CSV mode over EtherCAT/SOEM, Laifual L70I-E-100-BF) with
MoveIt 2 in a hybrid setup.

---

## Goal

Drive **joint 1** of the mycobot_280 with the real EtherCAT plugin, and
keep **joints 2-6 + gripper** on `mock_components/GenericSystem`. Plan
and execute trajectories from MoveIt's RViz panel against this hybrid
hardware. End state: a single `arm_controller` (JointTrajectoryController)
takes a position trajectory from MoveIt and outputs **velocity** to all
6 arm joints — joint 1 over EtherCAT, joints 2-6 to mock.

---

## Architecture

```
RViz MotionPlanning panel
        │  (FollowJointTrajectory action)
        ▼
moveit_simple_controller_manager
        │
        ▼
arm_controller (JTC)            joint_state_broadcaster
   joints: link1..link6                │
   command: velocity                   ▼
   state:   position, velocity     /joint_states
        │
        ▼
ros2_control resource manager
        │                                 │
        ▼                                 ▼
ros2_control name="…_real_arm"     ros2_control name="…_mock_arm"
plugin: mycobot_csv/MyCobotCSV     plugin: mock_components/GenericSystem
   joint: link1_to_link2              joints: link2_to_link3 … gripper
        │                                 │
        ▼                                 ▼
   EtherCAT (enxc8a3623069bf)         (in-process state echo /
   slave 1 (Laifual)                   velocity-integrating dynamics)
```

The two `<ros2_control>` blocks live in the **same** URDF; the resource
manager loads each as a separate hardware component. The arm controller
spans both blocks transparently.

---

## Key packages

| Package | Role | Edit policy |
|---|---|---|
| `mycobot_csv` | Verified ros2_control plugin (CSV/velocity). | **Do not modify** without explicit approval. See `src/mycobot_csv/docs/VERIFICATION.md`. |
| `mycobot_moveit_config` | MoveIt config + launch files. | Primary edit target. |
| `mycobot_description` | Clean geometry/kinematics URDF (no ros2_control). | Not modified. |
| `mycobot_gazebo` | Working gazebo+moveit baseline (reference only). | Not modified. |

---

## Files modified (and why)

| Path | What changed |
|---|---|
| `src/mycobot_ros2/mycobot_moveit_config/config/mycobot_280.ros2_control.xacro` | Replaced single block with two blocks: real-arm (mycobot_csv on `link1_to_link2`) + mock-arm (joints 2-6 + gripper). All 6 arm joints use velocity command interface; gripper stays on position. |
| `src/mycobot_ros2/mycobot_moveit_config/config/mycobot_280.urdf.xacro` | Switched the geometry include from `mycobot_gazebo/urdf/ros2_control/gazebo/mycobot_280.urdf` (had embedded `mycobot_hardware/MyCobotHardware` block) to `mycobot_description/urdf/mycobot_280_urdf.xacro` (clean — no ros2_control). |
| `src/mycobot_ros2/mycobot_moveit_config/config/ros2_controllers.yaml` | `command_interfaces: position → velocity`. Joint 1 gains lowered to `{p:5, d:1, i:0}` for safety; joints 2-6 left at original (mock — harmless). |
| `src/mycobot_ros2/mycobot_moveit_config/config/joint_limits.yaml` | `default_velocity_scaling_factor: 0.1 → 0.5`. Enabled `has_acceleration_limits: true`, `max_acceleration: 1.0` for `link1_to_link2`. |
| `src/mycobot_ros2/mycobot_moveit_config/launch/csv_moveit.py` | New launch file (MoveIt + hybrid hardware). Pins `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` via `SetEnvironmentVariable`. JSB spawner is `TimerAction(4.0)`; arm_controller spawner chains via `OnProcessExit(jsb)`. |
| `src/mycobot_ros2/mycobot_moveit_config/package.xml` | Added `<exec_depend>mycobot_description</exec_depend>` (kept `mycobot_gazebo` for baseline). |
| `src/mycobot_ros2/mycobot_moveit_config/config/mycobot_280.ros2_control.xacro.gazebo_backup` | Snapshot of the original (pre-Step-2) ros2_control xacro. |

The Gazebo baseline launch (`gazebo_moveit.py`) is **not** affected — it
points at `mycobot_gazebo/urdf/ros2_control/classic_gazebo/mycobot_280_gazebo_classic.urdf.xacro`
and never touches `mycobot_280.urdf.xacro`.

---

## Commits (chronological)

```
2944467  arm_controller: gentle PID gains for newly-real joint 2
d852084  Phase B1: move link2_to_link3 from mock_arm to real_arm
478d382  Zero-on-startup position offset for activation
a076feb  docs: INTEGRATION_NOTES — Phase 3 bench-verification + multi-slave commands
25865ad  mycobot_csv: Phase 3a — two-joint test scaffold
de66ba6  mycobot_csv: PDO size check supports multiple slaves
8eb85cd  Add INTEGRATION_NOTES.md — context, files, commands, open issues
7cc2001  Step 5d: initialize mock command buffers to prevent NaN state
532f4d6  Step 5c: bump default_velocity_scaling_factor 0.15 -> 0.5
1f5b00f  Step 5b: bump default_velocity_scaling_factor 0.05 -> 0.15
9ccb6e9  Step 5a: safer JTC gains, scaling, accel limits, RMW pin
08eb015  Step 4: csv_moveit.py launch file (MoveIt + hybrid CSV/mock hardware)
a61398d  Step 3: switch arm_controller to velocity command interface
52540a2  Step 2: hybrid CSV+mock hardware via mycobot_description URDF
072b293  Initial workspace state — pre-CSV+MoveIt integration
c5df6a6  Add .gitignore
```

To revert just Step 5d (the SIGABRT-causing commit):
```bash
git revert 7cc2001
```

---

## Steps status

| Step | What | Status |
|---|---|---|
| 1 | Source workspace, plugin discoverable | done |
| 2 | Update ros2_control.xacro (hybrid CSV+mock) + switch geometry include | done |
| 3 | `command_interfaces: position → velocity` | done |
| 4 | Create `csv_moveit.py` launch | done |
| 5 | Launch + verify hardware bring-up | done (joint 1 + 5 mock joints) |
| 6 | Plan + execute trajectory in RViz | done — joint 1 plans and executes on real hardware from RViz |
| Phase A | Bench-verify 2-motor multi-slave via `mycobot_csv` (no MoveIt) | done — see Multi-slave bench verification section |
| Phase B | Integrate joint 2 into MoveIt (move from mock-arm to real-arm block) | done — see "Phase B — adding joint 2 to MoveIt" section |
| Phase C | Repeat Phase B for joints 3, 4, 5, 6 (one at a time) | pending |

---

## How to build, source, launch

```bash
# Build
cd ~/mycobot_ws
colcon build --packages-select mycobot_moveit_config --symlink-install

# Source (every new shell)
source /opt/ros/humble/setup.bash
source install/setup.bash

# Launch (RMW export still recommended even though launch file pins it)
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch mycobot_moveit_config csv_moveit.py
```

Optional: tee logs for scrollback / Read-tool inspection:

```bash
ros2 launch mycobot_moveit_config csv_moveit.py 2>&1 | tee /tmp/csv_moveit.log
```

To verify the URDF expands cleanly (no plugin loading):

```bash
xacro $(ros2 pkg prefix mycobot_moveit_config)/share/mycobot_moveit_config/config/mycobot_280.urdf.xacro \
  > /tmp/mycobot_280.urdf
grep -c "<ros2_control" /tmp/mycobot_280.urdf   # expect 2
grep "<plugin>" /tmp/mycobot_280.urdf
# expect:
#   <plugin>mycobot_csv/MyCobotCSV</plugin>
#   <plugin>mock_components/GenericSystem</plugin>
```

---

## Pre-flight checklist before launching

1. **EtherCAT cap on ros2_control_node** (one time per install of `controller_manager`):
   ```bash
   sudo setcap cap_net_raw,cap_net_admin=eip \
     $(readlink -f $(ros2 pkg prefix controller_manager)/lib/controller_manager/ros2_control_node)
   # Verify:
   getcap $(readlink -f $(ros2 pkg prefix controller_manager)/lib/controller_manager/ros2_control_node)
   # Expect: cap_net_admin,cap_net_raw=eip
   ```
2. **EtherCAT cable** plugged into interface `enxc8a3623069bf` (run `ip link` to confirm).
3. **Drive powered**, green LED blinking (LED steady-on or off generally indicates a fault — power-cycle to recover).
4. **No stale ROS processes** holding the bus:
   ```bash
   pgrep -af "ros2_control_node|verify.launch|simple_ng|move_group|rviz2"
   ```

---

## Hardware reference values

| Param | Value | Source |
|---|---|---|
| EtherCAT interface | `enxc8a3623069bf` | `mycobot_280.ros2_control.xacro` (real-arm `<param name="ifname">`) |
| Joint 1 (link1_to_link2) slave_index | `1` | `mycobot_280.ros2_control.xacro` (real-arm `<param name="slave_index">`) |
| Joint 2 (link2_to_link3) slave_index | `2` | same file |
| Joint 1+2 counts_per_rad | `2085932.0` (placeholder) | identical for both; slave 2 likely needs different value (calibration TODO) |
| Position reporting semantics | **zero-on-startup** (deltas from activation pose) | `mycobot_csv.cpp` — `position_offset_counts_` captured in `on_activate`, subtracted in `read()` |
| Motor velocity hard limit (per joint) | `2.0 rad/s` | `<command_interface name="velocity">` `<param name="max">` in real-arm block |
| `default_velocity_scaling_factor` | `0.5` | `joint_limits.yaml` |
| `link1_to_link2.max_velocity` | `2.7925270` rad/s (URDF default) | `joint_limits.yaml` |
| `link1_to_link2.max_acceleration` | `1.0 rad/s²` | `joint_limits.yaml` |
| arm_controller gains, **real** joints (link1+link2) | `{p: 2.0, d: 0.05, i: 0.0}` | `ros2_controllers.yaml` — lowered after Phase B brought joint 2 online; aggressive d-term amplified encoder noise |
| arm_controller gains, **mock** joints (link3-link6) | `{p: 100, d: 10, i: 0.01}` | `ros2_controllers.yaml` (mock — harmless) |
| `controller_manager` `update_rate` | `100 Hz` | `ros2_controllers.yaml` |

### Speed-vs-safety math (with current values)

```
v_cmd = v_ff + p·err_pos + d·err_vel
v_ff_max = 2.79 × 0.5 = 1.40 rad/s    (≈ 80°/s)
worst-case (transient err_pos=0.05 rad, err_vel=0.10 rad/s):
   1.40 + 5·0.05 + 1·0.10 = 1.75 rad/s   (under 2.0 ✓)
worst-case (steady err_pos≤0.02, err_vel≤0.05):
   1.40 + 0.10 + 0.05 = 1.55 rad/s       (under 2.0 ✓)
```

To raise speed further, increase scaling toward `0.59` (approx ceiling
to keep transient worst < 2.0). Stop short of `0.6+` without measuring
real tracking errors on the bench.

---

## Design decisions — do not re-litigate

- All 6 arm joints use **velocity** command interface.
- **Single** arm_controller (JTC) manages all 6 joints across both hardware blocks.
- Gripper stays on **position** command interface (matches the GripperCommand action declared in `moveit_controllers.yaml`).
- Workspace consolidated outside docker.
- Real-arm block stays byte-compatible with the verified `test_one_joint.urdf` config (same params, same interfaces).

---

## Diagnostic commands

```bash
# Joint state — confirms hardware is publishing
ros2 topic echo --once /joint_states

# Controller manager state
ros2 control list_controllers
ros2 control list_hardware_components

# Active nodes (after launch)
ros2 node list

# Per-launch log directory (each run gets its own)
ls -t ~/.ros/log/ | head -3
# Each dir contains launch.log + per-node logs (ros2_control_node-1.log etc.)

# Stop everything
pkill -f "ros2 launch"   # or kill the parent shell
```

---

## Open issues — current state

### Step 5d (`7cc2001`) — formerly suspected SIGABRT trigger; empirically resolved

Originally observed: after commit `7cc2001` (`initialize mock command
buffers to prevent NaN state`), `ros2_control_node` immediately exited
with code -6 (SIGABRT) on `csv_moveit.py` launch.

Current state: that commit is still in the tree, and `csv_moveit.py`
launches cleanly through Phase B4 with both real joints active. The
original SIGABRT is no longer reproducing. Root cause was never
isolated — likely a transient interaction with bus state / process
state at the time, that the subsequent multi-slave PDO check fix
(`de66ba6`) and zero-on-startup offset (`478d382`) happened to mask.

If SIGABRT recurs, the original diagnosis paths still apply:
revert `7cc2001` and drop `calculate_dynamics:true` from the mock-arm
block; read `~/.ros/log/<latest>/ros2_control_node-1-stderr.log` for
the actual abort reason; or try alternate syntax for the
`initial_value` param.

### Pre-existing non-blocking errors (ignore)

These show up in every launch and are **not** caused by the integration
work. Listed here so they don't get re-investigated:

- `Group 'arm_with_gripper' is not a chain` — KDL kinematics solver
  can't handle the SRDF group that combines arm + gripper. The pure
  `arm` group works fine; only `arm_with_gripper` planning fails.
- `occupancy_map_monitor/PointCloudOctomapUpdater` plugin not found —
  `sensors_3d.yaml` references a depth-camera plugin that isn't
  installed. Ignored unless you actually want octomap.
- `Action server: /recognize_objects not available` — RViz expects the
  object-recognition action server; we never run one.

---

## File: package.xml additions

Note that `mycobot_moveit_config/package.xml` declares both the new and
old runtime URDF sources:

```xml
<exec_depend>mycobot_gazebo</exec_depend>      <!-- still used by gazebo_moveit.py baseline -->
<exec_depend>mycobot_description</exec_depend> <!-- new — used by csv_moveit.py via mycobot_280.urdf.xacro -->
```

---

## Verification protocol for `mycobot_csv` (independent of MoveIt)

If something goes wrong with joint 1 specifically, fall back to the
single-joint launch (`verify.launch.py`) or the two-joint launch
(`verify_two.launch.py`) and check against the protocol in
`src/mycobot_csv/docs/VERIFICATION.md`. If neither works, the bug is
upstream of the MoveIt integration.

```bash
ros2 launch mycobot_csv verify.launch.py
# Expect log lines:
#   MyCobotCSV initialised: ifname='enxc8a3623069bf', 1 joint(s), max slave 1.
#   ec_init on '<X>'...
#   Found 1 slave(s).
#   Slave 1: OPERATION_ENABLED, brake released. position_actual=<int>
#   MyCobotCSV ACTIVE — 1 drive(s) operating.
```

If this passes and `csv_moveit.py` doesn't, the bug is in the MoveIt
wrapping, not the plugin.

**Note: while two slaves are physically on the bus, `verify.launch.py`
(single-joint) will correctly fail with `PDO_SIZE_MISMATCH` — the
plugin's PDO size check now scales with `n_joints_`, and 1 declared
joint vs 2 mapped slaves is a real mismatch. Use `verify_two.launch.py`
in that situation.**

---

## Multi-slave bench verification (`verify_two`)

Phase A of multi-motor expansion: validate that the plugin handles two
EtherCAT slaves correctly **before** wiring the second motor into MoveIt.
Uses the raw `forward_velocity_controller` (no JTC, no MoveIt) so the
test only exercises the plugin's per-slave PDO mapping, not trajectory
control.

### Files (all under `src/mycobot_csv/`)

| File | Purpose |
|---|---|
| `urdf/test_two_joints.urdf` | 2-joint URDF, slaves 1 and 2, both Laifual L70I-E-100-BF placeholder calibration |
| `config/controllers_two.yaml` | JSB + `forward_velocity_controller` listing both joints, `update_rate: 200` |
| `launch/verify_two.launch.py` | rsp + ros2_control_node + JSB on `TimerAction(4.0)` + forward_velocity_controller chained on JSB exit |

The single-joint files (`test_one_joint.urdf`, `controllers.yaml`,
`verify.launch.py`) are **untouched** — they remain the verified
single-joint baseline.

### Launch

```bash
cd ~/mycobot_ws
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source install/setup.bash
ros2 launch mycobot_csv verify_two.launch.py 2>&1 | tee /tmp/verify_two.log
```

Expected bring-up log lines (in order):
```
EtherCAT: 2 slave(s) discovered.
Slave 1: OPERATION_ENABLED, brake released. position_actual=<int> counts.
Slave 2: OPERATION_ENABLED, brake released. position_actual=<int> counts.
MyCobotCSV ACTIVE — 2 drive(s) operating in CSV mode.
[spawner] Configured and activated joint_state_broadcaster
[spawner] Configured and activated forward_velocity_controller
```

If you see two WARN lines about `enable() in FAULT state (sw=0x1008),
resetting`, that is normal recovery — the plugin's `CW_FAULT_RESET`
ran successfully and the drives transitioned to OPERATION_ENABLED.

### Bench-test commands

In a second terminal (after the launch is up):

```bash
source ~/mycobot_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Helper: capture both joint positions cleanly (header + 2 values)
capture() {
  ros2 topic echo /joint_states --once 2>/dev/null \
    | grep -A2 "^position:" | head -3
}
```

#### Test 1 — joint_1 only

```bash
echo "=== Test 1: joint_1 only ==="
echo "BEFORE:" ; capture

ros2 topic pub --once /forward_velocity_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.05, 0.0]}"
sleep 2
ros2 topic pub --once /forward_velocity_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0]}"
sleep 0.3

echo "AFTER:" ; capture
```

Expected: joint_1 delta ≈ +0.15 rad, joint_2 unchanged (the +0.15 vs.
the nominal +0.10 = 0.05 rad/s × 2s is publish-overhead; `ros2 topic
pub --once` takes ~0.5 s to set up before publishing, so the actual
on-time is ~3 s).

#### Test 2 — joint_2 only

```bash
echo "=== Test 2: joint_2 only ==="
echo "BEFORE:" ; capture

ros2 topic pub --once /forward_velocity_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.0, 0.05]}"
sleep 2
ros2 topic pub --once /forward_velocity_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0]}"
sleep 0.3

echo "AFTER:" ; capture
```

Expected: joint_2 delta ≈ +0.15 rad, joint_1 unchanged.

#### Test 3 — both joints, opposite directions, simultaneously

```bash
echo "=== Test 3: both joints, opposite directions ==="
echo "BEFORE:" ; capture

ros2 topic pub --once /forward_velocity_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.05, -0.05]}"
sleep 2
ros2 topic pub --once /forward_velocity_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0]}"
sleep 0.3

echo "AFTER:" ; capture
```

Expected: joint_1 delta ≈ +0.15 rad, joint_2 delta ≈ −0.15 rad. Both
move simultaneously (within one PDO cycle).

### What the three tests prove

| Test | Demonstrates |
|---|---|
| 1 | joint_1 (slave 1) responds to its own command. joint_2 (slave 2) holds station when commanded zero. No cross-talk slave1 → slave2. |
| 2 | joint_2 (slave 2) responds to its own command. joint_1 (slave 1) holds station. No cross-talk slave2 → slave1. |
| 3 | Both slaves can be commanded in the same cycle. Direction signs honored independently. PDO mapping is correct per-slave. |

### Open calibration concern (slave 2)

Slave 2's reported encoder position at startup is on the order of
−1.7 × 10⁹ counts (vs. ~5 × 10⁶ for slave 1). With the placeholder
`counts_per_rad = 2085932.0` shared by both joints, slave 2's reported
absolute angle is ≈ −811 rad — clearly an artifact of the placeholder
calibration, not a real angle. **Visually, joint 1 rotates more than
joint 2 for the same numerical command** because slave 2's real
counts_per_rad differs from the placeholder. The numerical delta
matches the command magnitude on both joints (round-trip
self-consistency through the same wrong constant), but the *physical*
motion differs.

This is a calibration task, not a plugin bug. Address before tightening
trajectory tracking on joint 2 — a wrong `counts_per_rad` makes the
physical and commanded velocities differ by a constant factor, which
the JTC's PID will fight against.

### Stop / kill

The launch is long-lived. To stop:

```bash
# In the launch terminal: Ctrl-C
# Or from anywhere:
pkill -f verify_two.launch.py
```

---

## Phase B — adding joint 2 to MoveIt

**End state:** RViz can plan and execute trajectories involving both
real joints (`link1_to_link2` slave 1, `link2_to_link3` slave 2). The
remaining four arm joints + gripper are still on `mock_components`.
Use this section as a template when adding joints 3-6 in Phase C — the
sequence of failure modes here is what to expect for each.

### B1. Move `link2_to_link3` from mock-arm to real-arm

**Commit:** `d852084`
**File:** `src/mycobot_ros2/mycobot_moveit_config/config/mycobot_280.ros2_control.xacro`

Two parts in the same file:
1. **Add** a `<joint name="link2_to_link3">` block to the real-arm
   `<ros2_control>` block, modelled on the existing `link1_to_link2`
   entry. Fields: `slave_index=2`, `counts_per_rad=2085932.0`
   (placeholder, same as joint 1 — calibration TODO), velocity
   `command_interface` with `min=-2.0`, `max=2.0`, plus position +
   velocity state interfaces with `initial_value` from `initial_positions.yaml`.
2. **Remove** the corresponding `<joint name="link2_to_link3">` block
   from the mock-arm `<ros2_control>` block — claim ownership transfers
   from mock to real with no overlap.

Verification:
```bash
xacro $(ros2 pkg prefix mycobot_moveit_config)/share/mycobot_moveit_config/config/mycobot_280.urdf.xacro \
  > /tmp/mycobot_280_phaseB.urdf
grep -c '<ros2_control'   /tmp/mycobot_280_phaseB.urdf  # expect 2
grep slave_index           /tmp/mycobot_280_phaseB.urdf  # expect values 1 and 2
grep -c 'name="link2_to_link3"' /tmp/mycobot_280_phaseB.urdf  # expect 1
```

### B2. Build + URDF expansion check

```bash
colcon build --packages-select mycobot_moveit_config --symlink-install
source install/setup.bash
```

### B3 (first attempt). Launch — bring-up clean, joint 2 outside URDF limits

```bash
ros2 launch mycobot_moveit_config csv_moveit.py
```

Bring-up reached `MyCobotCSV ACTIVE — 2 drive(s) operating`,
`joint_state_broadcaster` and `arm_controller` activated, RViz opened.
But `/joint_states` showed:

```
link1_to_link2:  2.568 rad
link2_to_link3: -811.05 rad   ← outside URDF limit ±2.879793 rad
```

`link2_to_link3`'s URDF limit is `lower=-2.879793, upper=2.879793` (≈
±π). Slave 2's absolute encoder reads ~-1.7×10⁹ counts; divided by the
placeholder `counts_per_rad=2085932` that gives ~-811 rad. **MoveIt
would refuse to plan from a state outside joint limits — Phase B4 was
blocked.**

This was *not* a `counts_per_rad` calibration miss — to bring -1.7×10⁹
counts into ±π we'd need `counts_per_rad ≈ 1.7×10⁹`, three orders of
magnitude beyond plausible. Slave 2 has a multi-turn absolute encoder
that's wound far in one direction; no single calibration constant
lands it inside ±π.

### B-fix. Zero-on-startup position offset (plugin change)

**Commit:** `478d382`
**Files:** `src/mycobot_csv/include/mycobot_csv/mycobot_csv.hpp`,
`src/mycobot_csv/src/mycobot_csv.cpp`

Implemented "zero-on-startup" semantics: capture each joint's
`position_actual` at the end of `on_activate()` and subtract it from
all subsequent reads. Reported position becomes a *delta from
activation pose*, starting at 0.

Five-edit summary (all minimal):
1. **Header**: add member `std::vector<int32_t> position_offset_counts_;`
2. **`on_init`**: zero-initialize the new vector with size `n_joints_`.
3. **`on_activate`** (between `enable_drive` loop and ACTIVE log): one
   explicit `fieldbus_roundtrip()` to refresh `txpdo_`, then for each
   joint capture `position_offset_counts_[i] = txpdo_[s]->position_actual`.
   Log each capture: `Joint <i> (slave <s>): position offset captured = <N> counts`.
4. **`on_activate` init-read**: subtract the offset before storing
   `last_position_counts_` and computing `hw_positions_` (so JSB's
   first published message is consistent with subsequent `read()` cycles).
5. **`read()`**: subtract the offset from `txpdo_[s]->position_actual`
   before converting counts to radians. The velocity calculation
   (delta-based) is offset-invariant.

Single-slave path (1 joint, 1 slave) is unchanged in behavior:
`position_offset_counts_[0]` captures whatever the encoder reads at
activation and subtracts it — for a freshly-zeroed test rig that's 0
and the math is identical to before.

Regression-tested via `verify_two.launch.py`:
- Both joints report ~0 rad at startup (microradian residuals from
  PDO-cycle noise between offset capture and first `read()`).
- Phase 3d Test 1 (`{0.05, 0.0}` for 2 s) still gives joint_1 delta
  ≈ +0.15 rad, joint_2 unchanged.

### B3 (redo). Launch with offset fix

Logs now include the per-joint capture lines:

```
Joint 0 (slave 1): position offset captured = 5662772 counts
Joint 1 (slave 2): position offset captured = -1691793728 counts
MyCobotCSV ACTIVE — 2 drive(s) operating in CSV mode.
```

`/joint_states` reports both real joints near 0 rad. RViz shows the
robot in a sane pose. MoveIt is willing to plan.

### B4 (first attempt). Joint 2 jerks during arm_controller activation

When the JTC claimed `link2_to_link3/velocity` and started running its
control loop, joint 2's motor jerked violently — before any RViz plan
was sent. Diagnosis: `link2_to_link3` had inherited the mock-default
gains `{p:100, d:10, i:0.01}` from when it was a mock joint. With
`d=10`, the JTC's derivative term amplifies encoder ticking noise into
big velocity-command spikes the moment the controller activates.

### B-tune. Lower PID gains for both real joints

**Commit:** `2944467`
**File:** `src/mycobot_ros2/mycobot_moveit_config/config/ros2_controllers.yaml`

Both real joints now use:

```yaml
link1_to_link2: {p: 2.0, d: 0.05, i: 0.0}
link2_to_link3: {p: 2.0, d: 0.05, i: 0.0}
```

- `p=2`: 1 rad position error → 2 rad/s velocity command (right at the
  drive's hard limit). Worst typical transient: `1.40 + 0.10 + 0.005
  ≈ 1.5 rad/s` — comfortably under 2.0.
- `d=0.05`: 20× lower than joint 1's previous `d=1`, 200× lower than
  the mock-default `d=10`. Cuts derivative-amplified encoder noise
  proportionally.
- `i=0`: prevents integrator wind-up during slow tracking.

Mock joints 3-6 keep `{p:100, d:10, i:0.01}` — `mock_components` has
no real dynamics, aggressive gains are harmless there.

### B4 (success). RViz plan + execute on joint 2

After all three commits in this section landed (`d852084`, `478d382`,
`2944467` — plus the implicit dependency on the earlier multi-slave
PDO check `de66ba6`), `csv_moveit.py` launches cleanly and RViz can
plan and execute small trajectories on either real joint. Joint 1 and
joint 2 both move on commanded plan; mock joints 3-6 + gripper render
at their initial poses.

### Lessons that should generalize to Phase C (joints 3-6)

1. **`counts_per_rad` placeholders are not safe to copy across drives.**
   Each new slave needs treating as un-calibrated until verified. The
   zero-on-startup offset masks the absolute-position issue but does
   *not* fix the gain of angle reporting.
2. **Mock-default gains are toxic on real drives.** When migrating a
   joint from mock to real, drop its arm_controller PID gains to
   `{p:2, d:0.05, i:0}` *in the same commit* as the URDF migration —
   saves a power-cycle of the drive after a violent jerk.
3. **Capture the position offset in `on_activate`, not `on_init`.**
   The encoder is only readable once the bus is OPERATIONAL and the
   drive is enabled. `on_init` runs before any of that.
4. **Verify each migration via `verify_two.launch.py` first** (or
   `verify_three.launch.py` etc. as joints accumulate). Raw
   forward_velocity_controller bench tests catch plugin-level
   regressions without MoveIt's stack on top — easier to diagnose.
