import math
from geometry_msgs.msg import Pose
from moveit_msgs.msg import Constraints, JointConstraint, OrientationConstraint


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


def orientation_constraint(ee_link, frame_id, orientation, tol=0.05) -> Constraints:
    oc = OrientationConstraint()
    oc.header.frame_id = frame_id
    oc.link_name = ee_link
    oc.orientation = orientation
    oc.absolute_x_axis_tolerance = tol
    oc.absolute_y_axis_tolerance = tol
    oc.absolute_z_axis_tolerance = tol
    oc.weight = 1.0
    return Constraints(orientation_constraints=[oc])
