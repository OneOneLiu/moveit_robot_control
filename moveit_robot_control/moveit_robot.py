import math
import threading
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import (Constraints, JointConstraint, PositionConstraint,
                              OrientationConstraint, RobotState)
from moveit_msgs.srv import GetPositionFK, GetPositionIK, GetCartesianPath
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive


class MoveitRobot:

    def __init__(self, config):
        if isinstance(config, str):
            with open(config) as f:
                config = yaml.safe_load(f)

        self._group = config['planning_group']
        self._ee = config['ee_link']
        self._frame = config['base_frame']
        self._joints = config.get('joint_names', [])
        self._vel = config.get('velocity_scaling', 0.5)
        self._acc = config.get('acceleration_scaling', 0.5)
        self._plan_time = config.get('planning_time', 5.0)
        self._attempts = config.get('num_attempts', 3)
        self._pipeline = config.get('planner_pipeline', 'ompl')
        self._planner = config.get('planner_id', 'RRTConnect')
        self._path_constraints = Constraints()
        self._js = None  # latest JointState

        if not rclpy.ok():
            rclpy.init()
        self._node = Node('moveit_robot_' + self._group.replace('/', '_'))
        exe = SingleThreadedExecutor()
        exe.add_node(self._node)
        threading.Thread(target=exe.spin, daemon=True).start()

        self._move = ActionClient(self._node, MoveGroup, '/move_action')
        self._exec = ActionClient(self._node, ExecuteTrajectory, '/execute_trajectory')
        self._fk = self._node.create_client(GetPositionFK, '/compute_fk')
        self._ik = self._node.create_client(GetPositionIK, '/compute_ik')
        self._cart = self._node.create_client(GetCartesianPath, '/compute_cartesian_path')
        self._node.create_subscription(JointState, '/joint_states',
                                       lambda m: setattr(self, '_js', m), 10)

        self._move.wait_for_server(timeout_sec=10.0)
        self._exec.wait_for_server(timeout_sec=10.0)

    # ── internal ──────────────────────────────────────────────────────────

    def _svc(self, client, req, timeout=10.0):
        client.wait_for_service(timeout_sec=timeout)
        ev, box = threading.Event(), [None]
        def cb(f): box[0] = f.result(); ev.set()
        client.call_async(req).add_done_callback(cb)
        ev.wait(timeout=timeout)
        return box[0]

    def _action(self, client, goal, timeout=60.0):
        ev, box = threading.Event(), [None]
        def on_result(f): box[0] = f.result().result; ev.set()
        def on_goal(f):
            h = f.result()
            if h.accepted:
                h.get_result_async().add_done_callback(on_result)
            else:
                ev.set()
        client.send_goal_async(goal).add_done_callback(on_goal)
        ev.wait(timeout=timeout)
        return box[0]

    def _ok(self, result):
        return result is not None and result.error_code.val == 1

    def _base_goal(self, vel=None, acc=None, pipeline=None, planner=None,
                   planning_time=None, num_attempts=None):
        goal = MoveGroup.Goal()
        r = goal.request
        r.group_name = self._group
        r.max_velocity_scaling_factor = vel if vel is not None else self._vel
        r.max_acceleration_scaling_factor = acc if acc is not None else self._acc
        r.allowed_planning_time = planning_time if planning_time is not None else self._plan_time
        r.num_planning_attempts = num_attempts if num_attempts is not None else self._attempts
        r.pipeline_id = pipeline if pipeline is not None else self._pipeline
        r.planner_id = planner if planner is not None else self._planner
        r.path_constraints = self._path_constraints
        return goal

    def _joints_constraints(self, positions):
        c = Constraints()
        for name, pos in zip(self._joints, positions):
            c.joint_constraints.append(JointConstraint(
                joint_name=name, position=float(pos),
                tolerance_above=0.001, tolerance_below=0.001, weight=1.0))
        return c

    def _goal_constraints(self, target: Pose, lock_rx=True, lock_ry=True, lock_rz=True):
        """Tight sphere + orientation at target — goes in goal_constraints[], not path_constraints."""
        _tol = 0.01
        pc = PositionConstraint()
        pc.header.frame_id = self._frame
        pc.link_name = self._ee
        pc.constraint_region.primitives.append(
            SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[_tol])
        )
        pc.constraint_region.primitive_poses.append(target)
        pc.weight = 1.0

        oc = OrientationConstraint()
        oc.header.frame_id = self._frame
        oc.link_name = self._ee
        oc.orientation = target.orientation
        oc.absolute_x_axis_tolerance = _tol if lock_rx else math.pi
        oc.absolute_y_axis_tolerance = _tol if lock_ry else math.pi
        oc.absolute_z_axis_tolerance = _tol if lock_rz else math.pi
        oc.weight = 1.0

        return Constraints(position_constraints=[pc], orientation_constraints=[oc])

    # ── motion ────────────────────────────────────────────────────────────

    def move_to_joints(self, positions, **kw) -> bool:
        goal = self._base_goal(**kw)
        goal.request.goal_constraints = [self._joints_constraints(positions)]
        return self._ok(self._action(self._move, goal))

    def plan_to_joints(self, positions, **kw):
        goal = self._base_goal(**kw)
        goal.request.goal_constraints = [self._joints_constraints(positions)]
        goal.planning_options.plan_only = True
        r = self._action(self._move, goal)
        return r.planned_trajectory if self._ok(r) else None

    def move_to_pose(self, target: Pose, **kw) -> bool:
        goal = self._base_goal(**kw)
        goal.request.goal_constraints = [self._goal_constraints(target)]
        return self._ok(self._action(self._move, goal))

    def plan_to_pose(self, target: Pose, **kw):
        goal = self._base_goal(**kw)
        goal.request.goal_constraints = [self._goal_constraints(target)]
        goal.planning_options.plan_only = True
        r = self._action(self._move, goal)
        return r.planned_trajectory if self._ok(r) else None

    def move_to(self, x=None, y=None, z=None, **kw) -> bool:
        """Move EE to (x, y, z); omitted axes stay at their current value."""
        cur = self.get_ee_pose()
        if cur is None:
            return False
        target = Pose()
        target.position.x = float(cur.position.x if x is None else x)
        target.position.y = float(cur.position.y if y is None else y)
        target.position.z = float(cur.position.z if z is None else z)
        target.orientation = cur.orientation
        return self.move_cartesian([target], **kw)

    def move_cartesian(self, waypoints, max_step=0.01, **kw) -> bool:
        traj = self.plan_cartesian(waypoints, max_step=max_step, **kw)
        return self.execute(traj) if traj is not None else False

    def plan_cartesian(self, waypoints, max_step=0.01, **kw):
        req = GetCartesianPath.Request()
        req.header.frame_id = self._frame
        req.group_name = self._group
        req.link_name = self._ee
        req.waypoints = list(waypoints)
        req.max_step = float(max_step)
        req.jump_threshold = float(kw.get('jump_threshold', 0.0))
        req.avoid_collisions = kw.get('avoid_collisions', True)
        req.max_velocity_scaling_factor = kw.get('vel', self._vel)
        req.max_acceleration_scaling_factor = kw.get('acc', self._acc)
        if self._js is not None:
            req.start_state.joint_state = self._js
        resp = self._svc(self._cart, req)
        if resp is None or resp.fraction < 0.99:
            frac = resp.fraction if resp else 0.0
            self._node.get_logger().warning(f'Cartesian path incomplete: {frac:.2%}')
            return None
        return resp.solution

    def execute(self, trajectory) -> bool:
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        return self._ok(self._action(self._exec, goal))

    # ── state ─────────────────────────────────────────────────────────────

    def get_joint_positions(self) -> dict:
        if self._js is None:
            return {}
        return dict(zip(self._js.name, self._js.position))

    def get_ee_pose(self) -> Pose:
        if self._js is None:
            return None
        req = GetPositionFK.Request()
        req.header.frame_id = self._frame
        req.fk_link_names = [self._ee]
        req.robot_state.joint_state = self._js
        resp = self._svc(self._fk, req)
        if resp and resp.error_code.val == 1:
            return resp.pose_stamped[0].pose
        return None

    def compute_ik(self, target: Pose) -> dict:
        req = GetPositionIK.Request()
        req.ik_request.group_name = self._group
        req.ik_request.ik_link_name = self._ee
        req.ik_request.pose_stamped.header.frame_id = self._frame
        req.ik_request.pose_stamped.pose = target
        req.ik_request.avoid_collisions = True
        resp = self._svc(self._ik, req)
        if resp and resp.error_code.val == 1:
            js = resp.solution.joint_state
            return dict(zip(js.name, js.position))
        return None

    # ── settings ──────────────────────────────────────────────────────────

    def set_velocity_scaling(self, scale: float):
        self._vel = float(scale)

    def set_accel_scaling(self, scale: float):
        self._acc = float(scale)

    def set_planner(self, pipeline: str, planner_id: str):
        self._pipeline, self._planner = pipeline, planner_id

    def set_planning_time(self, seconds: float):
        self._plan_time = float(seconds)

    def set_path_constraints(self, constraints: Constraints):
        """Set path constraints applied to every planned trajectory.

        Build the Constraints object with functions from moveit_robot_control.constraints,
        then call this method before planning. Call clear_path_constraints() when done.
        """
        self._path_constraints = constraints

    def clear_path_constraints(self):
        self._path_constraints = Constraints()

    def get_robot_state(self) -> RobotState:
        """Current robot state as a RobotState message."""
        rs = RobotState()
        if self._js is not None:
            rs.joint_state = self._js
        return rs

    def get_trajectory_ee_poses(self, traj) -> list:
        """Compute FK for every point in a planned trajectory, return list of EE Pose.

        Used to visualize a trajectory as a sequence of EE positions — pass the result
        to Visualizer.trajectory_points().
        """
        poses = []
        names = list(traj.joint_trajectory.joint_names)
        for point in traj.joint_trajectory.points:
            req = GetPositionFK.Request()
            req.header.frame_id = self._frame
            req.fk_link_names = [self._ee]
            req.robot_state.joint_state.name = names
            req.robot_state.joint_state.position = list(point.positions)
            resp = self._svc(self._fk, req)
            if resp and resp.error_code.val == 1:
                poses.append(resp.pose_stamped[0].pose)
        return poses

    @property
    def node(self):
        return self._node

    @property
    def base_frame(self):
        return self._frame

    @property
    def ee_link(self):
        return self._ee
