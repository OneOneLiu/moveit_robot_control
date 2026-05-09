from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, Quaternion


def box(center: Pose, dims: tuple, frame_id: str, link_name: str,
        weight: float = 1.0) -> Constraints:
    """Path constraint: EE stays within a box volume for the entire trajectory.

    Args:
        center:    Center pose of the box (position + orientation of the box axes).
        dims:      (dx, dy, dz) full box extents in metres (total size, not half).
        frame_id:  Reference frame, e.g. robot.base_frame.
        link_name: Constrained link, e.g. robot.ee_link.
    """
    pc = PositionConstraint()
    pc.header.frame_id = frame_id
    pc.link_name = link_name
    pc.constraint_region.primitives.append(
        SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(dims))
    )
    pc.constraint_region.primitive_poses.append(center)
    pc.weight = weight
    return Constraints(position_constraints=[pc])


def plane(center: Pose, normal_quat: Quaternion, frame_id: str, link_name: str,
          thickness: float = 0.0005, weight: float = 1.0) -> Constraints:
    """Path constraint: EE stays on a plane (OMPL equality constraint).

    Creates a box whose Y-axis (in box-local frame after rotation by normal_quat)
    is the plane normal. The thin dimension (thickness) must satisfy:
        1e-4 < thickness < 1e-3
    The lower bound is OMPL's feasibility tolerance; the upper bound is the
    threshold below which OMPL treats the dimension as an equality constraint.
    0.0005 is the canonical safe value from the MoveIt documentation.

    Args:
        center:      A pose whose position is on the plane.
        normal_quat: Rotation that aligns the box Y-axis with the plane normal.
    """
    pc = PositionConstraint()
    pc.header.frame_id = frame_id
    pc.link_name = link_name
    pc.constraint_region.primitives.append(
        SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[1.0, thickness, 1.0])
    )
    pose = Pose()
    pose.position = center.position
    pose.orientation = normal_quat
    pc.constraint_region.primitive_poses.append(pose)
    pc.weight = weight
    c = Constraints(position_constraints=[pc])
    c.name = 'use_equality_constraints'
    return c


def line(center: Pose, dir_quat: Quaternion, frame_id: str, link_name: str,
         thickness: float = 0.0005, weight: float = 1.0) -> Constraints:
    """Path constraint: EE stays on a line (OMPL equality constraint).

    Creates a box whose Z-axis (after rotation by dir_quat) is the line direction.
    Both transverse dimensions are set to thickness (see plane() for range rules).

    Args:
        center:   A pose whose position lies on the line.
        dir_quat: Rotation that aligns the box Z-axis with the line direction.
    """
    pc = PositionConstraint()
    pc.header.frame_id = frame_id
    pc.link_name = link_name
    pc.constraint_region.primitives.append(
        SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[thickness, thickness, 1.0])
    )
    pose = Pose()
    pose.position = center.position
    pose.orientation = dir_quat
    pc.constraint_region.primitive_poses.append(pose)
    pc.weight = weight
    c = Constraints(position_constraints=[pc])
    c.name = 'use_equality_constraints'
    return c


def keep_orientation(quat: Quaternion, tol_xyz: tuple, frame_id: str, link_name: str,
                     weight: float = 1.0) -> Constraints:
    """Path constraint: EE maintains a fixed orientation throughout the trajectory.

    Args:
        quat:      Target orientation quaternion (e.g. from robot.get_ee_pose().orientation).
        tol_xyz:   (tol_x, tol_y, tol_z) angular tolerances in radians per axis.
                   Tight (e.g. 0.05) keeps orientation nearly fixed;
                   loose (e.g. 0.4) allows moderate drift.
        frame_id:  Reference frame.
        link_name: Constrained link.
    """
    oc = OrientationConstraint()
    oc.header.frame_id = frame_id
    oc.link_name = link_name
    oc.orientation = quat
    oc.absolute_x_axis_tolerance = float(tol_xyz[0])
    oc.absolute_y_axis_tolerance = float(tol_xyz[1])
    oc.absolute_z_axis_tolerance = float(tol_xyz[2])
    oc.weight = weight
    return Constraints(orientation_constraints=[oc])


def combine(*constraints: Constraints) -> Constraints:
    """Merge multiple Constraints objects into one (mixed constraints).

    Use when you need both a position and an orientation path constraint active
    simultaneously. If any input has name='use_equality_constraints', the output
    inherits it (last non-empty name wins).
    """
    result = Constraints()
    for c in constraints:
        result.position_constraints.extend(c.position_constraints)
        result.orientation_constraints.extend(c.orientation_constraints)
        result.joint_constraints.extend(c.joint_constraints)
        result.visibility_constraints.extend(c.visibility_constraints)
        if c.name:
            result.name = c.name
    return result
