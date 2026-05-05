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
7cc2001  Step 5d: initialize mock command buffers to prevent NaN state   ← CAUSES SIGABRT, see Open Issues
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
| 5 | Launch + verify hardware bring-up | partial — see Open Issues |
| 6 | Plan + execute trajectory in RViz | pending |

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
| Joint 1 slave index | `1` | same file (`<param name="slave_index">`) |
| Joint 1 counts_per_rad | `2085932.0` | placeholder, **needs calibration** before trusting velocity-command magnitudes |
| Joint 1 motor velocity hard limit | `2.0 rad/s` | `<command_interface name="velocity">` `<param name="max">` |
| `default_velocity_scaling_factor` | `0.5` | `joint_limits.yaml` |
| `link1_to_link2.max_velocity` | `2.7925270` rad/s (URDF default) | `joint_limits.yaml` |
| `link1_to_link2.max_acceleration` | `1.0 rad/s²` | `joint_limits.yaml` |
| arm_controller gains link1 | `{p: 5.0, d: 1.0, i: 0.0}` | `ros2_controllers.yaml` |
| arm_controller gains links 2-6 | `{p: 100, d: 10, i: 0.01}` | `ros2_controllers.yaml` (mock — harmless) |
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

### Step 5d caused SIGABRT in `ros2_control_node`

Commit `7cc2001` added `<param name="initial_value">0.0</param>` inside
each `<command_interface>` of the mock-arm block, intending to fix RViz
NaN-orientation errors caused by `mock_components/GenericSystem`
integrating an uninitialized command buffer when `calculate_dynamics:
true` is set.

After that commit, `ros2_control_node` started but immediately exited
with code -6 (SIGABRT). Bring-up never reached the controller spawn
phase. The launch's other processes (move_group, rviz2,
robot_state_publisher) came up fine — only the controller_manager
aborted.

**Most likely cause:** the installed Humble version of
`mock_components/GenericSystem` does not accept `<param
name="initial_value">` on `<command_interface>` — older releases
silently ignored it; some versions assert / abort on unknown params.

**Open paths to try next:**
1. **Revert Step 5d** (`git revert 7cc2001`) and instead drop the
   `<param name="calculate_dynamics">true</param>` line from the
   mock-arm block. Position state for joints 2-6 will then stay at
   their `initial_value` (0.0) regardless of commands — RViz
   visualization renders fine, joint 1 still executes on real
   hardware, mock-joint trajectory tracking is cosmetic only.
2. **Read the actual SIGABRT log** before any code change:
   ```bash
   # After a (failed) launch:
   ls -t ~/.ros/log/ | head -1
   cat ~/.ros/log/<that-dir>/ros2_control_node-1-stdout.log
   cat ~/.ros/log/<that-dir>/ros2_control_node-1-stderr.log
   # Or grep the streamed log:
   grep -nE "abort|terminate|what\\(\\)|Error|throw|ParameterAlreadyDeclaredException" /tmp/csv_moveit.log
   ```
   The exact assertion string would confirm or refute the theory.
3. **Last resort:** reorder the mock-arm `<command_interface>` declarations
   to use a different syntax variant (e.g. `<param name="initial_value">0</param>`
   without the decimal, or as an attribute) that some versions accept.

The pre-Step-5d behavior — bring-up succeeds, joint 1 moves, RViz spams
NaN errors for downstream links — is functionally usable for joint-1
testing but ugly for trajectory planning in RViz.

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
verified single-joint launch and check against the protocol in
`src/mycobot_csv/docs/VERIFICATION.md`. If `verify.launch.py` doesn't
work, the bug is upstream of the MoveIt integration.

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
