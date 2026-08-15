"""
scene_setup.py — conveyor control and package spawning.

The static layout (conveyor tracks, UR10, gripper, table, container) is
authored in `simulation/scene.usd`. This module owns everything that has to
happen at runtime:

    ConveyorController   start / stop the belts from Python
    PackageSpawner       spawn boxes under a gap + queue-cap policy
    SceneContext         opens the stage and ties it all together

"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
from pxr import Gf, Sdf, UsdGeom, UsdPhysics

import omni.usd

from isaacsim.core.api import World
from isaacsim.core.utils.stage import open_stage, add_reference_to_stage
from isaacsim.core.prims import SingleRigidPrim as _RigidPrim
from isaacsim.core.prims import SingleXFormPrim as _XFormPrim

import sim_config as C


# ===========================================================================
# Helpers
# ===========================================================================
def _pose_matrix(pos, quat):
    """USD transform from a (position, quaternion) world pose.
    `quat` arrives from get_world_pose() as (w, x, y, z)."""
    q = Gf.Quatd(float(quat[0]),
                 Gf.Vec3d(float(quat[1]), float(quat[2]), float(quat[3])))
    m = Gf.Matrix4d()
    m.SetTransform(Gf.Rotation(q),
                   Gf.Vec3d(float(pos[0]), float(pos[1]), float(pos[2])))
    return m


# ===========================================================================
# Conveyor
# ===========================================================================
class ConveyorController:
    #Start and stop the belts.

    VAR_ATTR = "graph:variable:Velocity"

    def __init__(self, stage, track_paths=None):
        self.stage = stage
        self.tracks = []
        for path in (track_paths or C.TRACKS):
            graph = stage.GetPrimAtPath(f"{path}/ConveyorBeltGraph")
            belt = stage.GetPrimAtPath(f"{path}/Belt")
            if not graph.IsValid():
                print(f"[conveyor] no graph at {path} — skipping")
                continue
            self.tracks.append((path, graph, belt))
        print(f"[conveyor] {len(self.tracks)} tracks under control")
        self._speed = 0.0

    def set_speed(self, speed, tracks=None):
        """Set belt speed in m/s. `tracks` = list of indices, or None for all."""
        self._speed = speed
        targets = self.tracks if tracks is None else [self.tracks[i] for i in tracks]

        for _path, graph, belt in targets:
            if not C.FORCE_DIRECT_SURFACE_VELOCITY:
                attr = graph.GetAttribute(self.VAR_ATTR)
                if attr and attr.IsValid():
                    attr.Set(float(speed))
                    continue
            # Fallback: drive PhysX directly. Local +X is world -X because of
            # the track's 180 deg Z rotation, so positive speed still moves
            # packages toward -X.
            if belt.IsValid():
                belt.GetAttribute("physxSurfaceVelocity:surfaceVelocity").Set(
                    (float(speed), 0.0, 0.0))

    def start(self):
        self.set_speed(C.BELT_SPEED)

    def stop(self):
        self.set_speed(0.0)

    @property
    def speed(self):
        return self._speed


# ===========================================================================
# Packages
# ===========================================================================
class Package:
    """One spawned box.

    `prim_path` is the referenced asset root; `body_path` is the rigid body
    inside it. They are not the same prim — box.usd is an assembly, and only
    the body can be tracked or joined to.
    """

    def __init__(self, pkg_id, prim_path, body_path):
        self.id = pkg_id
        self.prim_path = prim_path
        self.body_path = body_path
        self.arrived = False       # reached the pick zone
        self.retired = False       # delivered or lost — frees a queue slot
        self._rb = _RigidPrim(body_path, name=f"{pkg_id}_body")

    def position(self):
        pos, _ = self._rb.get_world_pose()
        return np.asarray(pos, dtype=float)

    def pose(self):
        return self._rb.get_world_pose()

    def __repr__(self):
        return f"<Package {self.id} @ {np.round(self.position(), 3).tolist()}>"


class PackageSpawner:

    def __init__(self, stage, conveyor, world):
        self.stage = stage
        self.conveyor = conveyor
        self.world = world
        self.active = []
        self.retired = []
        self._count = 0

        if not os.path.exists(C.BOX_USD):
            raise FileNotFoundError(f"package asset not found: {C.BOX_USD}")

        UsdGeom.Scope.Define(stage, C.PACKAGE_ROOT)

        stray = stage.GetPrimAtPath(C.STRAY_BOX_PATH)
        if stray.IsValid():
            stray.SetActive(False)
            print(f"[spawner] stray prop {C.STRAY_BOX_PATH} deactivated")

    # -- policy ------------------------------------------------------------
    def _should_spawn(self):
        if len(self.active) >= C.MAX_ON_BELT:
            return False
        if not self.active:
            return True
        travelled = abs(self.active[-1].position()[0] - C.SPAWN_POS[0])
        return travelled >= C.PACKAGE_GAP

    def spawn(self):
        pkg_id = f"pkg_{self._count:03d}"
        dst = f"{C.PACKAGE_ROOT}/{pkg_id}"
        self._count += 1

        add_reference_to_stage(usd_path=C.BOX_USD, prim_path=dst)

        # box.usd is its own articulation. Welding one articulation to another
        # (the UR10) with a fixed joint is unreliable in PhysX, so demote each
        # copy to a plain rigid-body assembly — the internal fixed joints that
        # hold the label and tape on are unaffected.
        artic = self.stage.GetPrimAtPath(f"{dst}/{C.BOX_ARTICULATION_SUBPATH}")
        if artic.IsValid():
            try:
                artic.RemoveAPI(UsdPhysics.ArticulationRootAPI)
            except Exception as exc:            # pragma: no cover
                print(f"[spawner] could not drop ArticulationRootAPI: {exc}")

        _XFormPrim(dst, name=pkg_id,
                   position=np.array(C.SPAWN_POS, dtype=float),
                   scale=np.full(3, float(C.PACKAGE_SCALE)))

        body_path = f"{dst}/{C.BOX_BODY_SUBPATH}"
        body = self.stage.GetPrimAtPath(body_path)
        if not body.IsValid():
            raise RuntimeError(
                f"expected a rigid body at {body_path} — check "
                "BOX_BODY_SUBPATH in config.py against your box.usd")

        if C.PACKAGE_MASS is not None:
            UsdPhysics.MassAPI.Apply(body).CreateMassAttr().Set(float(C.PACKAGE_MASS))

        pkg = Package(pkg_id, dst, body_path)
        self.active.append(pkg)
        return pkg

    # -- per-step update ---------------------------------------------------
    def update(self):
        """Call once per sim step. Returns a list of (event, package_id)."""
        events = []

        for pkg in self.active:
            if pkg.retired:
                continue
            x, _y, z = pkg.position()

            if z < C.BELT_MIN_Z:                      # fell off the line
                pkg.retired = True
                events.append(("package_lost", pkg.id))
                continue

            if not pkg.arrived and x <= C.PICK_X:     # reached the pick zone
                pkg.arrived = True
                events.append(("package_arrived", pkg.id))
                if C.STOP_ON_ARRIVAL:
                    self.conveyor.stop()

        if self._should_spawn():
            events.append(("package_spawned", self.spawn().id))

        # Resume once nothing is waiting in the pick zone.
        if C.STOP_ON_ARRIVAL and self.conveyor.speed == 0.0:
            if not any(p.arrived and not p.retired for p in self.active):
                self.conveyor.start()

        # Retired packages stop counting toward MAX_ON_BELT. They stay in the
        # scene — a box sitting in the basket looks right in the video.
        still_active = []
        for pkg in self.active:
            (self.retired if pkg.retired else still_active).append(pkg)
        self.active = still_active

        return events

    def retire(self, pkg_id):
        for pkg in self.active:
            if pkg.id == pkg_id:
                pkg.retired = True
                return True
        return False


# ===========================================================================
# Scene context
# ===========================================================================
class SceneContext:
    """Opens scene.usd and exposes the runtime pieces."""

    def __init__(self, usd_path=None):
        usd_path = os.path.normpath(usd_path or C.SCENE_USD)
        if not os.path.exists(usd_path):
            raise FileNotFoundError(usd_path)
        print(f"[scene] opening {usd_path}")
        open_stage(usd_path)

        self.world = World(stage_units_in_meters=1.0)
        self.stage = omni.usd.get_context().get_stage()

        self.conveyor = ConveyorController(self.stage)
        self.spawner = PackageSpawner(self.stage, self.conveyor, self.world)

        self._gripper_closed = False
        self._attached_id = None
        self._tip = self._try_prim(C.GRIPPER_TIP, "gripper_tip")
        self._grasp_body = self._try_prim(C.GRASP_BODY, "grasp_body")

        self.gripper_prim = self.stage.GetPrimAtPath(C.GRIPPER_PATH)
        if not self.gripper_prim.IsValid():
            print(f"[scene] no SurfaceGripper at {C.GRIPPER_PATH}")
        else:
            self._configure_gripper()

        print(f"[scene] grasp mode: {C.GRASP_MODE}")

    @staticmethod
    def _try_prim(path, name):
        try:
            return _XFormPrim(path, name=name)
        except Exception:
            print(f"[scene] no prim at {path}")
            return None

    def _configure_gripper(self):
        for attr, value in (
            ("isaac:maxGripDistance",   C.GRIP_MAX_DISTANCE),
            ("isaac:coaxialForceLimit", C.GRIP_COAXIAL_FORCE),
            ("isaac:shearForceLimit",   C.GRIP_SHEAR_FORCE),
            ("isaac:retryInterval",     C.GRIP_RETRY_INTERVAL),
        ):
            a = self.gripper_prim.GetAttribute(attr)
            if a and a.IsValid():
                a.Set(float(value))

        held = self.gripper_prim.GetRelationship("isaac:grippedObjects")
        if held and held.GetTargets():
            stale = [str(t) for t in held.GetTargets()]
            held.ClearTargets(True)
            print(f"[gripper] cleared stale grippedObjects: {stale}")

    def reset(self):
        self.world.reset()
        self.conveyor.start()

    def step(self):
        """One sim step + spawner update. Returns the spawner's events."""
        self.world.step(render=True)
        if not self.world.is_playing():
            return []
        return self.spawner.update()

    # -- grasping ----------------------------------------------------------
    def set_gripper(self, closed):
        self._gripper_closed = closed

        if self.gripper_prim.IsValid():
            a = self.gripper_prim.GetAttribute("isaac:status")
            if a and a.IsValid():
                a.Set("Closed" if closed else "Open")   # keeps the cup animating

        if C.GRASP_MODE == "joint":
            self._attach_nearest() if closed else self._detach()

    def gripped_package_id(self):
        if C.GRASP_MODE == "joint":
            return self._attached_id

        rel = self.gripper_prim.GetRelationship("isaac:grippedObjects")
        targets = rel.GetTargets() if rel else []
        if targets:
            name = targets[0].name
            for pkg in self.spawner.active:
                if name in (pkg.id, os.path.basename(pkg.body_path)):
                    return pkg.id
            return name
        if not self._gripper_closed:
            return None
        pkg, dist = self.nearest_package()
        return pkg.id if pkg and dist <= C.GRIP_DETECT_RADIUS else None

    def nearest_package(self):
        #Closest live package to the gripper tip, and its distance.
       
        tip = self.tip_position()
        if tip is None:
            return None, None
        best, best_d = None, float("inf")
        for pkg in self.spawner.active:
            d = float(np.linalg.norm(pkg.position() - tip))
            if d < best_d:
                best, best_d = pkg, d
        return best, (None if best is None else best_d)

    def tip_position(self):
        if self._tip is None:
            return None
        try:
            pos, _ = self._tip.get_world_pose()
            return np.asarray(pos, dtype=float)
        except Exception:
            return None

    def _attach_nearest(self):
        if self._grasp_body is None:
            print(f"[grasp] no prim at {C.GRASP_BODY} — cannot attach")
            return None

        pkg, dist = self.nearest_package()
        if pkg is None:
            print("[grasp] no live packages to grab")
            return None
        if dist > C.GRIP_DETECT_RADIUS:
            print(f"[grasp] nearest is {pkg.id} at {dist:.3f} m — outside the "f"{C.GRIP_DETECT_RADIUS} m radius, not grabbing")
            return None

        self._detach()

        # Preserve the current relative pose, so the box does not snap to the
        # gripper origin the moment the joint appears.
        rel = _pose_matrix(*pkg.pose()) * _pose_matrix(*self._grasp_body.get_world_pose()).GetInverse()
        t, q = rel.ExtractTranslation(), rel.ExtractRotationQuat()

        joint = UsdPhysics.FixedJoint.Define(self.stage, C.GRASP_JOINT_PATH)
        joint.CreateBody0Rel().SetTargets([Sdf.Path(C.GRASP_BODY)])
        joint.CreateBody1Rel().SetTargets([Sdf.Path(pkg.body_path)])
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(t))
        joint.CreateLocalRot0Attr().Set(Gf.Quatf(float(q.GetReal()), Gf.Vec3f(q.GetImaginary())))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))

        self._attached_id = pkg.id
        print(f"[grasp] attached {pkg.id} at {dist:.3f} m")
        return pkg.id

    def _detach(self):
        if self.stage.GetPrimAtPath(C.GRASP_JOINT_PATH).IsValid():
            self.stage.RemovePrim(C.GRASP_JOINT_PATH)
        self._attached_id = None