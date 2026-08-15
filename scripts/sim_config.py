"""
config.py — every tunable value for the teleoperation pipeline, in one place.

Nothing in here executes Isaac Sim code, so it is safe to import from any
interpreter (including the plain Python you use for plotting).

Layout of the scene, in world coordinates:

      spawn                                              pick     robot  bin
    x = +0.6  ------------- flow direction (-X) ------->  -6.0    -6.3   -8.3
    |=========|============|============|============|
     Track     Track_01     Track_02     Track_03

Each conveyor track Xform carries a 180 deg rotation about Z, so the
IsaacConveyor node's local +X maps to world -X.
"""

import os

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
REPO_ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
SIM_DIR   = os.path.join(REPO_ROOT, "simulation")
DATA_DIR  = os.path.join(REPO_ROOT, "data")
PLOTS_DIR = os.path.join(REPO_ROOT, "plots")

SCENE_USD = os.path.join(SIM_DIR, "scene.usd")
BOX_USD   = os.path.join(SIM_DIR, "N01851 Cubebox-IsaacSim", "box.usd")

# ---------------------------------------------------------------------------
# Prim paths on the stage
# ---------------------------------------------------------------------------
ROBOT_PATH    = "/World/ur10"
EE_PRIM       = "/World/ur10/ee_link"
GRIPPER_PATH  = "/World/short_gripper/SurfaceGripper"
GRIPPER_TIP   = "/World/short_gripper/gripper_tip"
CONVEYOR_ROOT = "/World/Conveyer"
BASKET_PATH   = "/World/IsaacSim_asset_69e78f80d12b0a2caf5d1415"
PACKAGE_ROOT  = "/World/Packages"

# Legacy warehouse prop. Deactivated on startup if still present, so it can
# never be mistaken for a package.
STRAY_BOX_PATH = "/World/SM_CardBoxB_01_1043"

# Conveyor tracks, upstream -> downstream.
TRACKS = [
    f"{CONVEYOR_ROOT}/ConveyorTrack",       # x =  0   spawn end
    f"{CONVEYOR_ROOT}/ConveyorTrack_01",    # x = -2
    f"{CONVEYOR_ROOT}/ConveyorTrack_02",    # x = -4
    f"{CONVEYOR_ROOT}/ConveyorTrack_03",    # x = -6   pick end
]

# ---------------------------------------------------------------------------
# The package asset
# ---------------------------------------------------------------------------

BOX_ARTICULATION_SUBPATH = "textured_mesh_69e4b9f60876e0e5e541893e"
BOX_BODY_SUBPATH = f"{BOX_ARTICULATION_SUBPATH}/part_00_corrugated_cardboard"

PACKAGE_SCALE = 1.0    
PACKAGE_MASS  = None    
# ---------------------------------------------------------------------------
# Scene geometry
# ---------------------------------------------------------------------------
SPAWN_POS  = (-1.0, 0.0, 2.35)   # a little above the belt, so it settles
PICK_X     = -6.00                  # package has "arrived" once x <= this
BELT_MIN_Z = 1.50                   # below this, it fell off the line

# ---------------------------------------------------------------------------
# Spawn policy
# ---------------------------------------------------------------------------
BELT_SPEED      = 0.50   # m/s
PACKAGE_GAP     = 1.20   # m between consecutive packages (configurable)
MAX_ON_BELT     = 3      # queue cap — holds the feeder when the picker lags
STOP_ON_ARRIVAL = True   # halt the belt while a package waits in the pick zone


# Set True only if writing the OmniGraph variable has no effect on your build;
# bypasses the conveyor graph and drives PhysX surface velocity directly.
FORCE_DIRECT_SURFACE_VELOCITY = False

# ---------------------------------------------------------------------------
# Grasping
# ---------------------------------------------------------------------------
#   "joint"   -> create a PhysicsFixedJoint between GRASP_BODY and the nearest
#                package body. Deterministic; reports why it refused.
#   "surface" -> write isaac:status and let the SurfaceGripper extension pick
#                a body itself. Closer to real hardware, much harder to debug.
GRASP_MODE       = "joint"
GRASP_BODY       = EE_PRIM              # must be a body in the UR10 articulation
GRASP_JOINT_PATH = "/World/GraspJoint"
GRIP_DETECT_RADIUS = 0.40               # m, centre-to-centre (see notes below)

# Only used when GRASP_MODE == "surface".
GRIP_MAX_DISTANCE   = 0.05
GRIP_COAXIAL_FORCE  = 1000.0
GRIP_SHEAR_FORCE    = 1000.0
GRIP_RETRY_INTERVAL = 0.5

# ---------------------------------------------------------------------------
# Teleoperation
# ---------------------------------------------------------------------------
JOG_SPEED = 0.6      # rad/s at 1x
DRIVE_KP  = 1.0e6    # raise if the arm sags under gravity
DRIVE_KD  = 1.0e5    # raise if it oscillates or buzzes
LIFT_DZ   = 0.08     # metres of rise that counts as a "lift"

# shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3
HOME_Q = (0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0)

# Key bindings. These deliberately avoid Kit's own hotkeys -- do NOT use
# Q/W/E/R (viewport gizmos), F (frame selected), P ("Parent Prims"), or
# SPACE (play/pause, which would stop the sim mid-pick).
JOG_KEYS = [("A", "Z"), ("S", "X"), ("D", "C"),
            ("G", "B"), ("H", "N"), ("J", "M")]
KEY_GRIP_CLOSE = "L"
KEY_GRIP_OPEN  = "K"
KEY_HOME       = "I"
KEY_SLOWER     = "COMMA"
KEY_FASTER     = "PERIOD"
KEY_STATUS     = "SEMICOLON"
KEY_QUIT       = "ESCAPE"
DEBUG_KEYS     = False    # True -> print every key name the callback receives

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------
TELEMETRY_HZ = 10.0   # sample rate for the plotting stream
