# mycobot_csp

A `ros2_control` SystemInterface plugin for the Laifual L70I-E-100-BF over
EtherCAT in CiA 402 **CSP** mode (Cyclic Synchronous Position).

Sibling of the verified [`mycobot_csv`](../mycobot_csv) (CSV/velocity)
baseline. Same EtherCAT bring-up flow, same CiA 402 state machine, same
SOEM idioms — but the drive runs the position loop, so the host just
streams target positions and the joint servos to them.

## Why this exists

Driving a joint to a target via CSV (cyclic velocity) and "stop when close"
is open-loop on the host side: small encoder drift between cycles
accumulates, and there's no graceful "hold" state without an active
command. CSP delegates the position loop to the drive — host sends a
target, drive servos there and holds.

The CSV plugin is preserved untouched as a fallback; pick the right one
for your task by selecting the right plugin string in your URDF.

## Differences from mycobot_csv

| Aspect | CSV | CSP |
|---|---|---|
| Mode-of-operation (0x6060) | 9 | **8** |
| RxPDO 0x1602 entry 2 | 0x60FF target velocity | **0x607A target position** |
| Exported command interface | velocity | **position** |
| `write()` unit conversion | rad/s × counts/rad | **rad × counts/rad** |
| NaN safety in command path | NaN → 0 (= stand still) | **NaN → current encoder pos** |
| Idle / priming target | 0 (= stand still) | **last position_actual** |

The CSP "NaN safety" extends through priming, brake release, and
`disable_drive` — see `docs/VERIFICATION.md` for the full audit list.

## Build

```bash
# Prereq: SOEM installed as shared lib (same as mycobot_csv); already done
# if mycobot_csv builds.

cd ~/mycobot_ws
colcon build --packages-select mycobot_csp --symlink-install
source install/setup.bash
```

The `setcap` step on `ros2_control_node` is shared with mycobot_csv —
already in place if CSV bring-up has worked on this machine.

## Running (standalone, no MoveIt)

```bash
# Edit urdf/test_one_joint.urdf — confirm <param name="ifname">
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch mycobot_csp verify.launch.py
```

In another terminal — note the current position p0 first:

```bash
ros2 topic echo --once /joint_states   # read position[0] = p0

# Step +0.05 rad
ros2 topic pub --once /forward_position_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [<p0 + 0.05>]}"

# Back to start
ros2 topic pub --once /forward_position_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [<p0>]}"
```

See `docs/VERIFICATION.md` for the full three-phase verification protocol.

## What's NOT in this package

- **Multi-joint hardware tested.** Code supports it but has only been
  verified single-joint.
- **MoveIt integration.** Once standalone CSP passes, a `csp_moveit.py`
  launch + `mycobot_280.ros2_control.csp.xacro` are added to
  `mycobot_moveit_config` (no edits to the verified CSV files).
