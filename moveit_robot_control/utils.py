import math
from geometry_msgs.msg import Pose, Quaternion
from moveit_robot_control import constraints


def pose(x=0.0, y=0.0, z=0.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0) -> Pose:
    p = Pose()
    p.position.x, p.position.y, p.position.z = float(x), float(y), float(z)
    p.orientation.x, p.orientation.y = float(qx), float(qy)
    p.orientation.z, p.orientation.w = float(qz), float(qw)
    return p


def pose_from_rpy(x, y, z, roll, pitch, yaw) -> Pose:
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    return pose(x, y, z,
                qx=sr * cp * cy - cr * sp * sy,
                qy=cr * sp * cy + sr * cp * sy,
                qz=cr * cp * sy - sr * sp * cy,
                qw=cr * cp * cy + sr * sp * sy)


def plan_exec(robot, viz, traj, pause_after='continue') -> bool:
    """Show planned trajectory as spheres, execute on Enter, then clear markers.

    pause_after: label for the prompt shown after execution
                 ('continue', 'finish', …). Pass None to clear immediately.
    """
    if not traj:
        print('    Planning failed.')
        if pause_after is not None:
            input(f'    Press Enter to {pause_after}...')
        viz.clear()
        return False
    print(f'    Planned {len(traj.joint_trajectory.points)} points — showing in RViz.')
    viz.trajectory_points(robot.get_trajectory_ee_poses(traj))
    input('    Press Enter to execute...')
    ok = robot.execute(traj)
    print(f'    {"OK" if ok else "FAILED"}')
    if pause_after is not None:
        input(f'    Press Enter to {pause_after}...')
    viz.clear()
    return ok


def demo_box(robot, viz):
    """Box constraint: keep the EE inside a 3D box for the entire trajectory.

    Key concept — path_constraints vs goal_constraints:
      goal_constraints  → only checked at the final pose (where the robot ends up)
      path_constraints  → checked at EVERY intermediate waypoint along the path

    The box is a path_constraints/PositionConstraint. OMPL samples only states that
    satisfy it, so every trajectory point is guaranteed to be inside the box —
    not just the start and end. The grey box in RViz shows the allowed region.
    """
    print('\n── [1/3] Box constraint ────────────────────────────────────────')

    cur = robot.get_ee_pose()
    if cur is None:
        print('    Cannot get EE pose, skipping.')
        return

    # Target: +0.2 m in Y, -0.1 m in Z. Both start and goal must lie inside
    # the box — if either violates the path constraint, planning always fails.
    target = Pose()
    target.position.x = cur.position.x
    target.position.y = cur.position.y + 0.2
    target.position.z = cur.position.z - 0.1
    target.orientation = cur.orientation

    # dims = full extents [x, y, z] — same convention as SolidPrimitive.BOX.
    box_dims = (0.15, 0.45, 0.35)
    box_center = Pose()
    box_center.position.x = cur.position.x
    box_center.position.y = cur.position.y + 0.1
    box_center.position.z = cur.position.z - 0.05
    box_center.orientation.w = 1.0

    viz.sphere(cur,    (1.0, 0.0, 0.0, 1.0), ns='pts')
    viz.sphere(target, (0.0, 1.0, 0.0, 1.0), ns='pts')
    viz.box(box_center, box_dims, (0.7, 0.7, 0.7, 0.25), ns='box')

    robot.set_path_constraints(
        constraints.box(box_center, box_dims, robot.base_frame, robot.ee_link)
    )
    print('    Planning...')
    plan_exec(robot, viz, robot.plan_to_pose(target, vel=0.05))
    robot.clear_path_constraints()


def demo_orientation(robot, viz):
    """Orientation constraint: lock EE orientation for the entire trajectory.

    The path_constraints holds an OrientationConstraint, not a PositionConstraint.
    The planner must find a path where the EE never rotates more than tol per axis.

    Practical use: carrying a cup, tray, or any object that must stay level.
    Tolerance 0.1 rad ≈ 6° prevents visible tilt while keeping planning tractable.
    """
    print('\n── [2/3] Orientation constraint ───────────────────────────────')

    cur = robot.get_ee_pose()
    if cur is None:
        print('    Cannot get EE pose, skipping.')
        return

    target = Pose()
    target.position.x = cur.position.x - 0.05
    target.position.y = cur.position.y + 0.2
    target.position.z = cur.position.z
    target.orientation = cur.orientation

    viz.sphere(cur,    (1.0, 0.0, 0.0, 1.0), ns='pts')
    viz.sphere(target, (0.0, 1.0, 0.0, 1.0), ns='pts')

    # Lock orientation to the current EE orientation. Every joint configuration
    # along the path must keep the EE within ±0.1 rad of this orientation.
    robot.set_path_constraints(
        constraints.keep_orientation(
            cur.orientation, (0.1, 0.1, 0.1),
            robot.base_frame, robot.ee_link,
        )
    )
    print('    Planning...')
    plan_exec(robot, viz,
              robot.plan_to_pose(target, vel=0.05, planning_time=20.0, num_attempts=5))
    robot.clear_path_constraints()


def demo_plane(robot, viz):
    """Plane constraint (OMPL equality constraint): EE stays on a horizontal plane.

    Why a thin box instead of a real plane primitive:
      OMPL checks PositionConstraint box dimensions against two thresholds:
        > 0.001 m  → inequality (EE must be inside the box)
        < 0.001 m  → equality  (EE is projected onto that face)
      The equality mode is what "constrained to a plane" actually means.

    The magic number 0.0005 m (thickness in constraints.plane()):
        OMPL feasibility tolerance: 1e-4 m  ← must be strictly greater
        OMPL equality threshold:    1e-3 m  ← must be strictly less
      0.0005 sits safely between both: treated as equality, yet feasible.

    "use_equality_constraints" (set automatically by constraints.plane()) tells
    OMPL to use the equality-constrained state space for sampling.

    IMPORTANT: requires in ompl_planning.yaml for your planning group:
        enforce_constrained_state_space: true
        projection_evaluator: joints(joint1,joint2)
    """
    print('\n── [3/3] Plane constraint (OMPL equality) ──────────────────────')
    print('    Requires enforce_constrained_state_space: true in ompl_planning.yaml.')

    cur = robot.get_ee_pose()
    if cur is None:
        print('    Cannot get EE pose, skipping.')
        return

    # Target at the same Z — both endpoints must lie on the plane.
    target = Pose()
    target.position.x = cur.position.x
    target.position.y = cur.position.y + 0.2
    target.position.z = cur.position.z
    target.orientation = cur.orientation

    # Horizontal plane normal = world Z.
    # constraints.plane() creates a box whose local Y is the thin dimension.
    # Rotating local Y → world Z requires a 90° rotation around world X:
    #   quaternion = (sin(π/4), 0, 0, cos(π/4))
    normal_quat = Quaternion(x=math.sin(math.pi / 4), y=0.0, z=0.0,
                             w=math.cos(math.pi / 4))

    slab_pose = Pose()
    slab_pose.position.x = cur.position.x
    slab_pose.position.y = cur.position.y + 0.1
    slab_pose.position.z = cur.position.z
    slab_pose.orientation.w = 1.0
    viz.box(slab_pose, (0.6, 0.6, 0.002), (0.2, 0.4, 1.0, 0.3), ns='plane')
    viz.sphere(cur,    (1.0, 0.0, 0.0, 1.0), ns='pts')
    viz.sphere(target, (0.0, 1.0, 0.0, 1.0), ns='pts')

    robot.set_path_constraints(
        constraints.plane(cur, normal_quat, robot.base_frame, robot.ee_link)
    )
    print('    Planning...')
    # Equality constraints need more planning time than inequality constraints.
    plan_exec(robot, viz,
              robot.plan_to_pose(target, vel=0.05, planning_time=15.0),
              pause_after='finish')
    robot.clear_path_constraints()
