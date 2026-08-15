# Teleoperated Pick-and-Place in NVIDIA Isaac Sim

A conveyor line with a UR10 manipulator, driven by a human operator from the
keyboard, with full event and telemetry logging.

Built and tested on **Isaac Sim 5.1**.

---

## Repository layout

```
scripts/
  sim_config.py     all tunables and prim paths, in one place
  scene_setup.py    conveyor control + package spawner
  teleop.py         keyboard teleoperation interface (entry point)
  data_logger.py    event/telemetry logger, and the plotting pass
simulation/
  scene.usd                    the authored scene
  N01851 Cubebox-IsaacSim/     package asset
  N01753 Container-IsaacSim/   drop-zone bin asset
data/                          logged CSV from the runs
plots/                         generated figures
```

`sim_config.py` is an addition to the structure given in the brief. Every
constant — prim paths, belt speed, spawn gap, drive gains, key bindings — lives
there, so tuning never means hunting through logic.

It is named `sim_config`, not `config`, on purpose: Isaac Sim's bundled OpenCV
puts its own directory on `sys.path`, and that directory contains a `config.py`
which raises `NameError: name 'LOADER_DIR' is not defined` when imported bare.
Each module also prepends its own directory to `sys.path` for the same reason.

---

## Running it

```bash
# full teleoperation session (this is the one you want)
<isaac-sim>/python.bat scripts/teleop.py

# conveyor + spawner only, no robot — useful for checking the line
<isaac-sim>/python.bat scripts/scene_setup.py

# turn a finished run into figures (system Python, needs matplotlib)
python scripts/data_logger.py data/run_20260812_141530_events.csv --cycle 0
```

Use Isaac Sim's bundled interpreter (`python.bat` / `python.sh`) for the first
two — they need the `omni` modules. The plotting pass deliberately has no Isaac
dependency, so run it with whichever Python has matplotlib.

### Controls

| Key | Action | Key | Action |
|-----|--------|-----|--------|
| `a` / `z` | shoulder_pan ± | `g` / `b` | wrist_1 ± |
| `s` / `x` | shoulder_lift ± | `h` / `n` | wrist_2 ± |
| `d` / `c` | elbow ± | `j` / `m` | wrist_3 ± |
| `l` | close gripper (grasp) | `k` | open gripper (release) |
| `i` | return to home pose | `,` / `.` | jog slower / faster |
| `;` | status dump | `Esc` | quit |

Click the viewport first — keyboard events go to the focused window.

These bindings avoid Kit's own hotkeys on purpose. `Q`/`W`/`E`/`R` are the
viewport gizmos, `F` frames the selection, `P` is "Parent Prims", and `SPACE`
toggles play/pause — binding to any of those makes the application fight the
operator, and `SPACE` in particular would pause the simulation mid-pick.

---

## Scene walkthrough

### Assets

| Piece | Source |
|---|---|
| **Robot** — UR10, 6-DoF | Isaac Sim asset library (`Isaac/Robots/UniversalRobots/ur10`) |
| **Gripper** — `short_gripper`, vacuum type | Isaac Sim asset library, the UR10's own end effector |
| **Conveyor** — 4 × `ConveyorTrack` | Isaac Sim conveyor generator (`isaacsim.asset.gen.conveyor`) |
| **Packages** — cardboard box | `simulation/N01851 Cubebox-IsaacSim/box.usd` |
| **Drop zone** — wooden bin | `simulation/N01753 Container-IsaacSim/` |

Nothing was modelled from scratch.

**Why this gripper.** A vacuum end effector only has to reach the top face of
the box — there is no jaw alignment, no finger-friction tuning, and no grasp
width to get right. With keyboard jogging and no IK, a parallel-jaw gripper
would have meant fighting PhysX contact friction for every attempt. Suction
turns the pick into "get above it and descend", which is the part a human can
actually do through six key pairs.

An earlier attempt used the SO-101 arm. It was dropped for two reasons: it has
five arm DoF plus a gripper, not the six the brief asks for, and its ~0.35 m
reach is far too short for a conveyor-side pick. The UR10 also needs no URDF
import at all, which removed a whole category of failure.

### How the conveyor moves

Four `ConveyorTrack` prims are laid end to end along X. Each carries a 180°
rotation about Z, so the `IsaacConveyor` node's local **+X** maps to world
**−X**: packages spawn at x ≈ +0.6 and travel toward the robot at x ≈ −6.3.

Each track owns an OmniGraph whose `Velocity` variable feeds its conveyor
node, and that node rewrites `physxSurfaceVelocity:surfaceVelocity` on the belt
every tick. Writing the belt attribute directly gets overwritten — so
`ConveyorController` drives `graph:variable:Velocity` instead, which is the
supported hook. (`FORCE_DIRECT_SURFACE_VELOCITY` in `sim_config.py` falls back to
writing PhysX directly if that ever misbehaves.)

### How packages are spawned

`box.usd` is referenced onto the stage once per package, positioned at the head
of the line. Two conditions gate a spawn:

1. the most recent package has travelled at least `PACKAGE_GAP` (1.2 m) from
   the spawn point — the brief's configurable gap; and
2. fewer than `MAX_ON_BELT` (3) packages are live.

Condition 2 is what makes the line survive a human operator. On a fixed timer,
a fumbled pick means boxes stack up until the pick zone is unusable. With the
cap, the feeder simply holds until the picker catches up, then resumes — which
is how accumulation control works on a real line.

### How the teleoperation interface works

`teleop.py` owns one loop. Each pass steps physics, reads the set of currently
held keys, nudges a joint-target vector, and writes it to the articulation:

```python
for event, pkg_id in scene.step():        # physics + spawner
    emit(event, pkg_id)
...
q_target[dof] += JOG_SPEED * speed_scale * dt
arm.apply_action(ArticulationAction(joint_positions=q_target))
```

The keyboard callback never blocks — it only adds to and removes from a set.
Anything blocking there (a prompt, a sleep) freezes physics and the viewport.

Control is **joint-space only**: each key pair drives one motor. No IK, no
motion planning, no trajectory generation. The brief states smoothness is not
evaluated, so a solver would have added singularities and convergence failures
in exchange for nothing.

---

## Design decisions worth flagging

**The conveyor is an indexing conveyor, not a continuous one.** When a package
reaches the pick zone the belt stops; it restarts when the package is released
into the bin. Intercepting a moving box by jogging individual joints from a
keyboard is close to impossible, and letting the belt grind against a stopped
box makes it jitter and creep sideways exactly when you are trying to grab it.
Stopping the belt removes that failure mode and gives unambiguous event
boundaries in the log. Real pick stations work this way.

**Grasping uses an explicit fixed joint, not the SurfaceGripper's own
attachment.** `GRASP_MODE = "joint"` creates a `PhysicsFixedJoint` between
`ur10/ee_link` and the nearest package body, preserving the relative pose so
the box does not snap; release deletes the joint. The `SurfaceGripper` prim is
still driven (`isaac:status`) so the cup animates, but it is not what holds the
box.

The reason is debuggability. The SurfaceGripper API is implicit — you set
`isaac:status = "Closed"` and the extension decides for itself what to attach,
subject to `maxGripDistance` and two force limits. When it silently declines,
it gives no reason. The fixed joint either attaches or prints the distance it
measured and why it refused. Setting `GRASP_MODE = "surface"` in `sim_config.py`
switches back to the extension's own behaviour; both paths are implemented.

**Packages are demoted from articulations to rigid-body assemblies on spawn.**
`box.usd` ships as an articulation — a cardboard shell plus a label, a decal
and a strip of tape, welded on with fixed joints. Fixed-jointing one
articulation (the box) to another (the UR10) is unreliable in PhysX, so each
spawned copy has `PhysicsArticulationRootAPI` removed. The internal joints
holding the trim on are untouched. All physics interaction targets
`part_00_corrugated_cardboard`, the shell body that carries the mass.

**Nearest-package detection is centre-to-centre.** `GRIP_DETECT_RADIUS` is
0.30 m, which sounds generous until you note the box is ~0.31 m across: the
distance from the tip prim to the box's origin is ~0.16 m even when the cup is
resting on its top face.

---

## Logged data

Two CSVs per run, both carrying exactly the seven fields the brief specifies:

| File | Contents |
|---|---|
| `data/run_<stamp>_events.csv` | one row per event — the deliverable |
| `data/run_<stamp>_telemetry.csv` | sampled at 10 Hz — supporting data for the plots |

| Field | Description |
|---|---|
| `timestamp_sim` | Isaac Sim simulation time (seconds) |
| `timestamp_wall` | wall-clock time (unix epoch seconds) |
| `event` | `package_spawned`, `package_arrived`, `grasp`, `lift`, `drop`, `grip_lost`, `package_lost` |
| `joint_positions` | all joint angles at that moment, JSON list |
| `ee_position` | end-effector XYZ in world frame, JSON list |
| `gripper_state` | `open` / `closed` |
| `package_id` | which package is being handled |

They are split so the deliverable stays small and readable — a handful of rows
per cycle instead of thousands — while the plots still have a continuous
signal. Both share one schema and concatenate cleanly.

Cycle numbers are deliberately **not** a column: they are derived at plot time
by counting `drop` events, so nothing outside the specified schema is stored.

`grasp` is emitted only once a grasp is confirmed — the loop checks what is
actually attached rather than assuming the keypress worked, so the log never
claims a pick that did not happen. `lift` fires on the first 8 cm of rise after
a confirmed grasp.

A `run_<stamp>_meta.json` sidecar records the real joint names so figures are
labelled `shoulder_pan` rather than `j0`.

### Plots

`plots/cycle_N_joints_and_ee.png` — all six joint angles across one cycle with
`arrived` / `grasp` / `lift` / `drop` marked, and end-effector X, Y and Z as
three separate panels below.

`plots/cycle_N_ee_path.png` — the end-effector path through space for the same
cycle, with grasp and drop marked.

X, Y and Z get their own panels rather than sharing an axis: in this world
frame they sit at roughly −6, 0 and 2, and on a shared scale the Z motion —
which is the entire pick-and-place signal — flattens into a straight line. The
six joint colours are a fixed, colour-vision-checked order.

---

## Known limitations

- **Motion is jerky.** Joint-space jogging with no interpolation, by design.
- **The grasp is a rigid weld,** not simulated suction. It cannot slip, so this
  does not model vacuum loss on a bad seal or a heavy load.
- **Package spawning caps at 3 on the belt.** A faster operator would not be
  fed faster; the cap is a robustness choice, not a throughput model.
- **`PICK_X` is calibrated by observation.** If packages coast past the pick
  zone or stop short, that constant needs adjusting for the belt speed in use.
- The `surface` grasp mode is implemented but less tested than `joint` mode.

## Troubleshooting

Press `;` during a run for a status dump — sim time, whether the timeline is
playing, belt speed, how many packages have spawned, where each one is relative
to the gripper tip, and what is currently held. Nearly every failure shows up
there as either "0 packages spawned" (spawner or timeline problem) or a large
`dist_to_tip` (a reach problem), which points at the fix immediately.
