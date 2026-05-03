# mycobot_csv

A `ros2_control` SystemInterface plugin for the Laifual L70I-E-100-BF over
EtherCAT in CiA 402 **CSV** mode (Cyclic Synchronous Velocity).

This package is a **strict mirror** of `simple_ng.c`. Its purpose is to be
a *trusted baseline* — verified line-by-line against working code — before
migrating to CSP for MoveIt integration.

## Why this exists

We previously jumped from working `simple_ng.c` (CSV) to a CSP plugin in
one step. That plugin had drift problems and we couldn't easily tell whether
the bug was in (a) the ros2_control wrapping, (b) the CSV→CSP mode change,
or (c) the EtherCAT link itself.

This package isolates change (a). If this works on the bench, then we know
the ros2_control wrapping is correct, and any future CSP issue is isolated
to the (b)+(c) variables.

## Read this in order

1. **`docs/VERIFICATION.md`** — the verification protocol. Three phases
   (code review, build, runtime) with a checklist for each. Do all three
   before trusting this plugin.
2. **`include/mycobot_csv/mycobot_csv.hpp`** — the header. Short. Read it
   first before reading the implementation.
3. **`src/mycobot_csv.cpp`** — the implementation. Sections are tagged
   with simple_ng line ranges; verify each one.

## What's NOT in this package (intentionally)

- **Position command interface.** CSV mode commands velocity. If you want
  to move to a position via this plugin, use `joint_trajectory_controller`
  configured with velocity command interfaces — ros2_control's built-in
  position-to-velocity logic. We do not write our own position controller
  inside the plugin.
- **CSP mode.** That's the next package, after this one is verified.
- **Multi-joint support tested.** The code supports it (per-joint
  config, vectors), but verification is single-joint only. Don't trust
  multi-joint until single-joint is solid.
- **MoveIt integration.** That waits until the CSP version.

## Build

```bash
# Once: install SOEM as shared library
cd /path/to/SOEM/build
rm -rf *
cmake .. -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=/usr/local
cmake --build . -j$(nproc)
sudo cmake --install .
sudo ldconfig

# Verify SOEM:
ls /usr/local/lib/libsoem.so      # should exist
ldconfig -p | grep soem            # should list libsoem.so

# Then build the plugin in your colcon workspace:
cd ~/your_ws
colcon build --packages-select mycobot_csv --symlink-install
source install/setup.bash

# Once: cap_net_raw on ros2_control_node
sudo setcap cap_net_raw,cap_net_admin=eip \
  $(readlink -f $(ros2 pkg prefix controller_manager)/lib/controller_manager/ros2_control_node)
```

## Running

```bash
# Edit urdf/test_one_joint.urdf — set <param name="ifname"> to your interface
ros2 launch mycobot_csv verify.launch.py
```

Then in another terminal:

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp 
# Spin

ros2 topic pub --once /forward_velocity_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.2]}"

# Stop
ros2 topic pub --once /forward_velocity_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.0]}"
```

See `docs/VERIFICATION.md` for the full protocol.
