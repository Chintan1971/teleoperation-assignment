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

`sim_config.py` consists every tunable value for the teleoperation pipeline, in one place.

It is named `sim_config`, not `config`, on purpose: Isaac Sim's bundled OpenCV
puts its own directory on `sys.path`, and that directory contains a `config.py`
which raises `NameError: name 'LOADER_DIR' is not defined` when imported bare.
Each module also prepends its own directory to `sys.path` for the same reason.

---

## Running it

```bash
# full teleoperation session 
<isaac-sim>/python.bat scripts/teleop.py

# conveyor + spawner only, no robot 
<isaac-sim>/python.bat scripts/scene_setup.py

# turn a finished run into figures (system Python, needs matplotlib)
python scripts/data_logger.py data/run_20260812_141530_events.csv --cycle 0
```


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



## Troubleshooting

Press `;` during a run for a status dump — sim time, whether the timeline is
playing, belt speed, how many packages have spawned, where each one is relative
to the gripper tip, and what is currently held. Nearly every failure shows up
there as either "0 packages spawned" (spawner or timeline problem) or a large
`dist_to_tip` (a reach problem), which points at the fix immediately.
