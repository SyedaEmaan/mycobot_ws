# VERIFICATION.md — building trust in mycobot_csv

The whole point of this package is to be a **trusted baseline**. Trust comes
from three things, in order:

1. **Code-level equivalence to simple_ng.c** (read & verify by eye)
2. **Build-level cleanliness** (it compiles without warnings)
3. **Runtime equivalence to simple_ng** (does the same thing on the bench)

Do all three before trusting this plugin.

---

## Phase 1 — Code-level equivalence (30 minutes, no hardware needed)

Open `simple_ng.c` and `src/mycobot_csv.cpp` side-by-side in your editor of
choice (VS Code split view, vim splits, two terminals — whatever). The cpp
file is annotated with simple_ng line ranges like:

```cpp
/* === simple_ng.c:107-254  laifual_csv_config  (verbatim) === */
```

Walk through each section in the cpp file and confirm the marked simple_ng
range produces the same SDO/PDO/state-machine behavior. The checklist below
makes this concrete.

### Checklist (tick each one)

- [ ] **PDO struct layout (csv_rxpdo_t, csv_txpdo_t).**
      `mycobot_csv.hpp` lines 39-55 vs `simple_ng.c` lines 22-38.
      Field order, types, sizes (12 bytes / 26 bytes), `__attribute__((packed))` —
      all identical.

- [ ] **CiA 402 constants (CW_*, SW_*).**
      `mycobot_csv.hpp` lines 58-68 vs `simple_ng.c` lines 41-51. Identical.

- [ ] **slave_csv_config / laifual_csv_config — Step 1 (set CSV mode).**
      Object 0x6060, value 9, sleep 50ms. Identical.

- [ ] **slave_csv_config — Step 2 (interpolation period).**
      0x60C2:01 = 5, 0x60C2:02 = -3. Identical.

- [ ] **slave_csv_config — Step 3 (TxPDO 0x1A02 mapping).**
      Same 8 entries in same order:
      0x60410010, 0x60640020, 0x60770010, 0x60F40020,
      0x60B90010, 0x60BA0020, 0x60BC0020, 0x60FD0020.
      Same write-zero-then-fill-then-write-count idiom.

- [ ] **slave_csv_config — Step 4 (RxPDO 0x1602 mapping).**
      Same 4 entries in same order:
      0x60400010, 0x60FF0020, 0x60B80010, 0x60FE0120.

- [ ] **slave_csv_config — Step 4b (output mask 0x60FE:02 = 0x01).**
      Identical.

- [ ] **slave_csv_config — Step 5 (SyncManager assignment).**
      0x1C12 → 0x1602, 0x1C13 → 0x1A02. Same write-zero / fill / write-one idiom.

- [ ] **slave_csv_config — Step 6 (free-run sync).**
      0x1C32:01 = 0, 0x1C33:01 = 0. Identical.

- [ ] **fieldbus_roundtrip.**
      `ecx_send_processdata` → `ecx_receive_processdata(..., 10000)`. Identical.

- [ ] **init_soem (mirroring fieldbus_start setup portion).**
      ec_init → config_init → attach PO2SOconfig hook → config_map_group →
      disable SM watchdog (reg 0x0422). Order and content match.

- [ ] **PDO priming (15 cycles total before requesting OP).**
      `reach_operational()`: 5 + 10 = 15 roundtrips with safe values.
      Matches `simple_ng.c:385` (5) + `simple_ng.c:401` (10).

- [ ] **enable_drive — fault reset.**
      CW = 0x80 (CW_FAULT_RESET), poll up to 100×5ms.

- [ ] **enable_drive — Shutdown transition (CW = 0x06).**
      Poll up to 200×5ms for SW_READY_TO_SWITCH_ON.

- [ ] **enable_drive — Switch On transition (CW = 0x07).**
      Poll up to 200×5ms for SW_SWITCHED_ON.

- [ ] **enable_drive — Enable Operation transition (CW = 0x0F).**
      Poll up to 200×5ms for SW_OPERATION_ENABLED.

- [ ] **enable_drive — Brake release.**
      20 cycles of CW=0x0F, target_velocity=0, physical_output=0x01,
      5ms between cycles. Matches `simple_ng.c:603-615`.

- [ ] **write() per-cycle behavior.**
      Per-cycle send: rxpdo->target_velocity = X; controlword = 0x0F;
      physical_output = 0x01; touch_probe_func = 0; then roundtrip.
      Matches the body of simple_ng's main loop at lines 651-654 + 656.

If any item fails, **STOP** and find me. Don't continue to Phase 2 with
unresolved divergences.

### Things that are intentionally different

For the sake of full transparency, here are the only intentional deltas
between simple_ng.c and mycobot_csv.cpp:

1. **No DBG() printouts inside slave_csv_config.** simple_ng prints after
   every SDO operation; we keep the writes but not the prints. We trust
   that simple_ng already verified the writes succeed on this hardware.
   If a write fails, slave_csv_config returns -1, which gets surfaced
   later as a SAFE_OP failure.

2. **No SDO readbacks inside slave_csv_config.** simple_ng reads back
   every value to confirm it took. We skip the readbacks, again because
   simple_ng already verified them.

3. **The main loop becomes the ros2_control update loop.** simple_ng has
   its own `while (running)` loop that calls `osal_usleep(5000)` between
   cycles. In our plugin, ControllerManager calls `read()` and `write()`
   at `update_rate` Hz (set in controllers.yaml to 200 Hz, same as simple_ng).

4. **`target_velocity` is dynamic, not constant.** simple_ng commands a
   hardcoded `-700000`. Our plugin commands `hw_velocity_commands_[i] *
   counts_per_rad_[i]`, where `hw_velocity_commands_` is set by whatever
   ROS topic publishes to forward_velocity_controller.

5. **Logging is RCLCPP_INFO instead of printf.** Cosmetic.

6. **`fieldbus_check_state` and `target reached` stop logic from
   simple_ng's main are NOT ported.** They're not bring-up logic; they're
   recovery and demo logic. Skipping them keeps the verification surface
   smaller.

If you find any other difference I haven't listed here, that's a bug —
flag it.

---

## Phase 2 — Build cleanliness (5 minutes, no hardware)

```bash
cd ~/path/to/your/ws
colcon build --packages-select mycobot_csv --symlink-install
source install/setup.bash
```

Pass criteria:

- [ ] `Finished <<< mycobot_csv` with no `Failed` lines
- [ ] No `-Wpedantic` or `-Wextra` warnings about our code
  (warnings about SOEM headers are OK, those aren't ours)
- [ ] `ls install/mycobot_csv/lib/libmycobot_csv.so` exists
- [ ] `cat install/mycobot_csv/share/ament_index/resource_index/hardware_interface__pluginlib__plugin/mycobot_csv`
       prints `share/mycobot_csv/mycobot_csv_plugin.xml`

If any fail: build issue, fix before Phase 3.

---

## Phase 3 — Runtime equivalence (15 minutes, hardware needed)

### 3.1 — Bus brings up

```bash
# Edit urdf/test_one_joint.urdf first: set <param name="ifname"> to your
# actual EtherCAT interface (run `ip link` to find it).

ros2 launch mycobot_csv verify.launch.py
```

Pass criteria — log should contain, in order:

- [ ] `MyCobotCSV initialised: ifname='<X>', 1 joint(s), max slave 1.`
- [ ] `ec_init on '<X>'...`
- [ ] `Found 1 slave(s).`
- [ ] `Slave 1 enable entry: sw=0x0XXX (...)`
- [ ] `Slave 1: OPERATION_ENABLED, brake released. position_actual=<some int>`
- [ ] `MyCobotCSV ACTIVE — 1 drive(s) operating.`
- [ ] `[forward_velocity_controller] activate_successful` (after the 4s delay)

If bring-up fails before "ACTIVE", the failure mode is the same as it would
be in simple_ng.c — same SDO writes, same state machine — so simple_ng's
behavior on the same hardware is your reference. If simple_ng works and
this doesn't, the difference is in the ros2_control wrapping (a Phase 1
checklist item failed).

### 3.2 — Drive holds still when no command published

With the launch from 3.1 still running, in another terminal:

```bash
ros2 topic echo /joint_states
```

- [ ] `position` reads a stable value (encoder noise is OK; drift of >1
       count/sec is NOT)
- [ ] `velocity` reads ~0 (with some noise around zero)

This is the most important test of all. If the joint drifts here, the
plugin has the same bug as the CSP version had — the EtherCAT path is
unreliable, the drive is misbehaving, or some other root cause.

If this passes and CSP failed the same test, the bug was specific to
CSP — and we now have a trusted CSV baseline to do incremental migration
from.

### 3.3 — Drive moves when commanded velocity, stops when commanded zero

```bash
# Spin slowly: 0.2 rad/s
ros2 topic pub --once /forward_velocity_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.2]}"
```

- [ ] Joint rotates roughly 0.2 rad/s in one direction
- [ ] Position in `/joint_states` increases continuously
- [ ] No fault, statusword stays 0x0027

```bash
# Stop
ros2 topic pub --once /forward_velocity_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.0]}"
```

- [ ] Joint stops
- [ ] Position settles, no drift
- [ ] Returns to the same hold-still behavior as 3.2

```bash
# Reverse
ros2 topic pub --once /forward_velocity_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [-0.2]}"
```

- [ ] Joint rotates the other direction
- [ ] Position decreases continuously

### 3.4 — Cleanup

Ctrl-C the launch. Should see in the log:

- [ ] `Deactivating MyCobotCSV...`
- [ ] `EtherCAT socket closed.`

If you can power-cycle and re-launch without issue, on_deactivate is
working correctly.

---

## Outcome

When all three phases pass:

- The package is **verified**: same code as simple_ng, builds cleanly,
  behaves identically on hardware.
- You can **trust** it. Any future change starts from this known-good
  state.
- You're now in position to do CSP migration as a **single-variable
  change** (mode 9 → mode 8, target_velocity → target_position) with a
  trusted reference to diff against.

If any phase fails: STOP, document what failed, ping me. Do not work
around the failure. The whole point of this exercise is that we have a
trusted artifact at the end — no asterisks.
