"""
Controls
--------
  a/z   shoulder_pan          g/b   wrist_1
  s/x   shoulder_lift         h/n   wrist_2
  d/c   elbow                 j/m   wrist_3

  l     gripper CLOSE (grasp)           k   gripper OPEN (release)
  i     return to home pose
  , .   jog slower / faster
  ;     status dump
  ESC   quit

"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import os
import sys
import time

# Isaac Sim's bundled packages litter sys.path, so make sure this directory
# wins for our own module names.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
import carb.input
import omni.appwindow


from isaacsim.core.prims import SingleArticulation as Articulation
from isaacsim.core.prims import SingleXFormPrim as XFormPrim
from isaacsim.core.utils.types import ArticulationAction

import sim_config as C
from scene_setup import SceneContext
from data_logger import DataLogger


# ---------------------------------------------------------------------------
# Scene and robot
# ---------------------------------------------------------------------------
scene = SceneContext()

arm = Articulation(prim_path=C.ROBOT_PATH, name="ur10")
scene.world.scene.add(arm)
scene.reset()
arm.initialize()

joint_names = list(arm.dof_names)
n_dof = len(joint_names)
print(f"\n[teleop] {n_dof} DOFs: {joint_names}\n")
if n_dof == 0:
    raise RuntimeError(
        f"articulation at {C.ROBOT_PATH} has 0 DOFs — the robot did not load "
        "as an articulation. Check the payload resolved and physics is on.")

arm.get_articulation_controller().set_gains(
    kps=np.full(n_dof, C.DRIVE_KP), kds=np.full(n_dof, C.DRIVE_KD))

try:
    lower = np.asarray(arm.dof_properties["lower"], dtype=float)
    upper = np.asarray(arm.dof_properties["upper"], dtype=float)
    bad = ~np.isfinite(lower) | ~np.isfinite(upper) | (lower >= upper)
    lower[bad], upper[bad] = -2 * np.pi, 2 * np.pi
except Exception:
    lower, upper = np.full(n_dof, -2 * np.pi), np.full(n_dof, 2 * np.pi)

ee = XFormPrim(C.EE_PRIM, name="ee")
logger = DataLogger(joint_names=joint_names)


def ee_position():
    pos, _ = ee.get_world_pose()
    return np.asarray(pos, dtype=float)


# ---------------------------------------------------------------------------
# State
# ---------------------------------------------------------------------------
held = set()
should_quit = False
go_home = False
grip_cmd = None            # None | True (close) | False (open)
speed_scale = 1.0

gripper_closed = False
holding = None             # package_id currently gripped
grasp_z = None             # EE height at the moment of grasp
lifted = False


# ---------------------------------------------------------------------------
# Events — one funnel for everything loggable
# ---------------------------------------------------------------------------
def make_row(event, package_id=None):
    return {
        "timestamp_sim":   scene.world.current_time,
        "timestamp_wall":  time.time(),
        "event":           event,
        "joint_positions": arm.get_joint_positions().tolist(),
        "ee_position":     ee_position().tolist(),
        "gripper_state":   "closed" if gripper_closed else "open",
        "package_id":      package_id,
    }

def emit(event, package_id=None):
    row = make_row(event, package_id)
    print(f"  [{row['timestamp_sim']:8.2f}s] {event:18s} {package_id or ''}")
    logger.log(row)

def status():
    """Diagnostic dump: is the sim playing, is the belt moving, are packages
    spawning, where is the tip relative to them, is anything gripped."""
    sp = scene.spawner
    tip = scene.tip_position()
    print("\n" + "=" * 62)
    print(f"  sim time      : {scene.world.current_time:.2f} s")
    print(f"  playing       : {scene.world.is_playing()}")
    print(f"  belt speed    : {scene.conveyor.speed}")
    print(f"  spawned total : {sp._count}   active: {len(sp.active)}   "
          f"retired: {len(sp.retired)}")
    print(f"  gripper closed: {gripper_closed}   holding: {holding}")
    print(f"  gripper tip   : {None if tip is None else np.round(tip, 3).tolist()}")
    print(f"  ee_link       : {np.round(ee_position(), 3).tolist()}")
    print(f"  cycles logged : {logger.cycles}")
    if not sp.active:
        print("  !! no active packages — nothing to grasp")
    for pkg in sp.active:
        p = pkg.position()
        d = "-" if tip is None else f"{np.linalg.norm(p - tip):.3f}"
        print(f"    {pkg.id}  pos={np.round(p, 3).tolist()}  "
              f"dist_to_tip={d}  arrived={pkg.arrived}")
    print(f"  q = {np.round(arm.get_joint_positions(), 3).tolist()}")
    print("=" * 62 + "\n")


# ---------------------------------------------------------------------------
# Keyboard — event driven, non-blocking
# ---------------------------------------------------------------------------
def on_keyboard(event, *args):
    global should_quit, speed_scale, go_home, grip_cmd
    key = event.input.name

    if event.type == carb.input.KeyboardEventType.KEY_PRESS:
        if C.DEBUG_KEYS:
            print(f"[key] {key}")
        held.add(key)
        if key == C.KEY_QUIT:
            should_quit = True
        elif key == C.KEY_GRIP_CLOSE:
            grip_cmd = True
        elif key == C.KEY_GRIP_OPEN:
            grip_cmd = False
        elif key == C.KEY_HOME:
            go_home = True
        elif key == C.KEY_SLOWER:
            speed_scale = max(0.1, speed_scale * 0.5)
            print(f"[teleop] speed x{speed_scale:.2f}")
        elif key == C.KEY_FASTER:
            speed_scale = min(4.0, speed_scale * 2.0)
            print(f"[teleop] speed x{speed_scale:.2f}")
        elif key == C.KEY_STATUS:
            status()
    elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
        held.discard(key)
    return True


app_window = omni.appwindow.get_default_app_window()
input_iface = carb.input.acquire_input_interface()
kb_sub = input_iface.subscribe_to_keyboard_events(app_window.get_keyboard(), on_keyboard)
print(__doc__)


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------
q_target = arm.get_joint_positions().copy()
home_q = np.asarray(C.HOME_Q, dtype=float)
dt = 1.0 / 60.0
last_sample = -1.0
last_warn = 0.0

while simulation_app.is_running() and not should_quit:
    # scene.step() runs world.step() plus the spawner, and returns its events
    for event, pkg_id in scene.step():
        emit(event, pkg_id)

    if not scene.world.is_playing():
        continue

    now = scene.world.current_time

    # --- arm jogging ------------------------------------------------------
    for dof, (up_key, dn_key) in enumerate(C.JOG_KEYS[:n_dof]):
        delta = 0.0
        if up_key in held:
            delta += C.JOG_SPEED * speed_scale * dt
        if dn_key in held:
            delta -= C.JOG_SPEED * speed_scale * dt
        if delta:
            q_target[dof] = np.clip(q_target[dof] + delta, lower[dof], upper[dof])

    if go_home:
        q_target = home_q[:n_dof].copy()
        go_home = False
        print("[teleop] returning to home pose")

    arm.apply_action(ArticulationAction(joint_positions=q_target))

    # --- gripper ----------------------------------------------------------
    if grip_cmd is not None:
        scene.set_gripper(grip_cmd)
        gripper_closed = grip_cmd
        if not grip_cmd and holding is not None:
            emit("drop", holding)                  # released over the basket
            scene.spawner.retire(holding)          # frees a queue slot
            holding, grasp_z, lifted = None, None, False
        grip_cmd = None

    # --- did the gripper actually catch something? ------------------------
    if gripper_closed and holding is None:
        caught = scene.gripped_package_id()
        if caught:
            holding, grasp_z, lifted = caught, ee_position()[2], False
            emit("grasp", holding)

    # --- lift: first meaningful rise after a confirmed grasp --------------
    if holding is not None and not lifted:
        if ee_position()[2] - grasp_z >= C.LIFT_DZ:
            lifted = True
            emit("lift", holding)

    # --- lost the box -----------------------------------------------------
    if holding is not None and gripper_closed and not scene.gripped_package_id():
        emit("grip_lost", holding)
        holding, grasp_z, lifted = None, None, False

    # --- telemetry --------------------------------------------------------
    if now - last_sample >= 1.0 / C.TELEMETRY_HZ:
        last_sample = now
        logger.sample(make_row("sample", holding))

    # --- heartbeat if nothing is spawning ---------------------------------
    if now - last_warn > 10.0:
        last_warn = now
        if scene.spawner._count == 0:
            print(f"[warn] {now:.0f}s of sim and 0 packages spawned. "
                  f"Press '{C.KEY_STATUS.lower()}' for a status dump.")

logger.close()
input_iface.unsubscribe_to_keyboard_events(app_window.get_keyboard(), kb_sub)
simulation_app.close()
