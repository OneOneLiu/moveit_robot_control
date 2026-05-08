import threading
import time
import xml.etree.ElementTree as ET
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rcl_interfaces.srv import GetParameters
from moveit_msgs.msg import (CollisionObject, AttachedCollisionObject,
                              PlanningScene as PlanningSceneMsg, PlanningSceneComponents,
                              AllowedCollisionMatrix, AllowedCollisionEntry)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene, LoadGeometryFromFile, SaveGeometryToFile
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive


class PlanningScene:

    def __init__(self, base_frame='world', node=None):
        self._frame = base_frame
        self._reference_acm = None  # lazily captured before first world modification

        if node is not None:
            self._node = node
            self._owns = False
        else:
            if not rclpy.ok():
                rclpy.init()
            self._node = Node('planning_scene_manager')
            exe = SingleThreadedExecutor()
            exe.add_node(self._node)
            threading.Thread(target=exe.spin, daemon=True).start()
            self._owns = True

        # Services for all scene operations and persistence
        self._apply = self._node.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self._get = self._node.create_client(GetPlanningScene, '/get_planning_scene')
        self._load = self._node.create_client(LoadGeometryFromFile, '/load_geometry_from_file')
        self._save = self._node.create_client(SaveGeometryToFile, '/save_geometry_to_file')
        self._get_params = self._node.create_client(GetParameters, '/move_group/get_parameters')

        # Eagerly push the SRDF ACM to move_group.
        # rviz2 publishes full scenes (is_diff=False) with an empty ACM to /planning_scene,
        # which causes move_group to wipe the SRDF disabled-collision pairs from its parent
        # scene.  Restoring the ACM here ensures it is correct before any planning request.
        self._ensure_ref_acm()
        if self._reference_acm is not None:
            _init_scene = PlanningSceneMsg(is_diff=True)
            _init_scene.allowed_collision_matrix = self._reference_acm
            self._apply_diff(_init_scene)

    # ── internal ──────────────────────────────────────────────────────────

    def _svc(self, client, req, timeout=5.0):
        if not client.wait_for_service(timeout_sec=timeout):
            self._node.get_logger().warning(f'Service {client.srv_name} not available')
            return None
        ev, box = threading.Event(), [None]
        def cb(f): box[0] = f.result(); ev.set()
        client.call_async(req).add_done_callback(cb)
        ev.wait(timeout=timeout)
        return box[0]

    def _apply_diff(self, scene: PlanningSceneMsg) -> bool:
        req = ApplyPlanningScene.Request()
        req.scene = scene
        resp = self._svc(self._apply, req)
        return resp is not None and resp.success

    def _apply_with_acm(self, scene: PlanningSceneMsg) -> bool:
        """Apply a planning scene diff and atomically restore the reference ACM.

        Including the ACM in the same ApplyPlanningScene call ensures that even
        if MoveIt2 resets ACM entries while processing the geometry diff, they
        are overwritten by the reference ACM in the same transaction (geometry
        is processed before ACM inside setPlanningSceneDiffMsg).
        """
        self._ensure_ref_acm()
        if self._reference_acm is not None:
            scene.allowed_collision_matrix = self._reference_acm
        return self._apply_diff(scene)

    def _make_object(self, name, pose: Pose, primitive: SolidPrimitive) -> CollisionObject:
        obj = CollisionObject()
        obj.header.frame_id = self._frame
        obj.id = name
        obj.primitives = [primitive]
        obj.primitive_poses = [pose]
        obj.operation = CollisionObject.ADD
        return obj

    def _get_acm(self):
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        resp = self._svc(self._get, req)
        return resp.scene.allowed_collision_matrix if resp else None

    def _build_acm_from_srdf(self) -> AllowedCollisionMatrix | None:
        """Read robot_description_semantic from move_group and build the ACM directly.

        This is a fallback for MoveIt2 Jazzy where the planning scene's ACM can be
        empty even though the SRDF was loaded correctly as a parameter.
        """
        try:
            if not self._get_params.wait_for_service(timeout_sec=5.0):
                return None
            req = GetParameters.Request()
            req.names = ['robot_description_semantic']
            resp = self._svc(self._get_params, req, timeout=5.0)
            if resp is None or not resp.values:
                return None
            srdf_string = resp.values[0].string_value
            if not srdf_string:
                return None

            root = ET.fromstring(srdf_string)
            disabled_pairs: list[tuple[str, str]] = []
            for dc in root.findall('disable_collisions'):
                l1, l2 = dc.get('link1'), dc.get('link2')
                if l1 and l2:
                    disabled_pairs.append((l1, l2))

            if not disabled_pairs:
                return None

            # Build a compact NxN AllowedCollisionMatrix from the disabled pairs.
            link_names: list[str] = []
            for l1, l2 in disabled_pairs:
                for ln in (l1, l2):
                    if ln not in link_names:
                        link_names.append(ln)

            n = len(link_names)
            idx = {name: i for i, name in enumerate(link_names)}
            matrix = [[False] * n for _ in range(n)]
            for l1, l2 in disabled_pairs:
                i, j = idx[l1], idx[l2]
                matrix[i][j] = matrix[j][i] = True

            acm = AllowedCollisionMatrix()
            acm.entry_names = link_names
            for row in matrix:
                entry = AllowedCollisionEntry()
                entry.enabled = row
                acm.entry_values.append(entry)
            return acm
        except Exception as exc:
            self._node.get_logger().warning(f'PlanningScene: could not build ACM from SRDF: {exc}')
            return None

    def _ensure_ref_acm(self):
        """Capture the SRDF-populated ACM; fall back to parsing the SRDF parameter
        when MoveGroup's live ACM is empty (a known MoveIt2 Jazzy initialisation gap)."""
        if self._reference_acm is not None and self._reference_acm.entry_names:
            return
        acm = self._get_acm()
        if acm and acm.entry_names:
            self._reference_acm = acm
        else:
            self._reference_acm = self._build_acm_from_srdf()

    def _apply_co(self, obj: CollisionObject, timeout=5.0) -> bool:
        """Apply a CollisionObject change and confirm it is reflected in the scene.

        Uses ApplyPlanningScene (synchronous service) with the reference ACM
        bundled in the same diff to prevent ACM corruption from the geometry
        update from persisting.
        """
        is_add = obj.operation != CollisionObject.REMOVE
        scene = PlanningSceneMsg(is_diff=True)
        scene.world.collision_objects = [obj]
        if not self._apply_with_acm(scene):
            return False
        deadline = time.time() + timeout
        while time.time() < deadline:
            names = self.list_objects()
            if is_add and obj.id in names:
                return True
            if not is_add and obj.id not in names:
                return True
            time.sleep(0.05)
        return False

    # ── add primitives ────────────────────────────────────────────────────

    def add_box(self, name: str, pose: Pose, size) -> bool:
        prim = SolidPrimitive(type=SolidPrimitive.BOX,
                              dimensions=[float(size[0]), float(size[1]), float(size[2])])
        return self._apply_co(self._make_object(name, pose, prim))

    def add_sphere(self, name: str, pose: Pose, radius: float) -> bool:
        prim = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[float(radius)])
        return self._apply_co(self._make_object(name, pose, prim))

    def add_cylinder(self, name: str, pose: Pose, radius: float, height: float) -> bool:
        prim = SolidPrimitive(type=SolidPrimitive.CYLINDER,
                              dimensions=[float(height), float(radius)])
        return self._apply_co(self._make_object(name, pose, prim))

    # ── manage objects ────────────────────────────────────────────────────

    def remove_object(self, name: str) -> bool:
        obj = CollisionObject()
        obj.header.frame_id = self._frame
        obj.id = name
        obj.operation = CollisionObject.REMOVE
        return self._apply_co(obj)

    def attach_object(self, name: str, link: str, touch_links=None) -> bool:
        attached = AttachedCollisionObject()
        attached.link_name = link
        attached.object.id = name
        attached.object.operation = CollisionObject.ADD
        attached.touch_links = touch_links or []
        scene = PlanningSceneMsg(is_diff=True)
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = [attached]
        return self._apply_with_acm(scene)

    def detach_object(self, name: str) -> bool:
        attached = AttachedCollisionObject()
        attached.object.id = name
        attached.object.operation = CollisionObject.REMOVE
        scene = PlanningSceneMsg(is_diff=True)
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = [attached]
        return self._apply_with_acm(scene)

    def list_objects(self) -> list:
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES
        resp = self._svc(self._get, req)
        if resp:
            return [obj.id for obj in resp.scene.world.collision_objects]
        return []

    def clear(self) -> bool:
        for name in list(self.list_objects()):
            self.remove_object(name)
        return True

    # ── persistence ───────────────────────────────────────────────────────

    def save(self, path: str) -> bool:
        req = SaveGeometryToFile.Request()
        req.file_path_and_name = path
        resp = self._svc(self._save, req)
        return resp is not None and resp.success

    def load(self, path: str) -> bool:
        req = LoadGeometryFromFile.Request()
        req.file_path_and_name = path
        resp = self._svc(self._load, req)
        return resp is not None and resp.success
