from geometry_msgs.msg import Pose, Vector3
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy

_LATCHED = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)


class Visualizer:
    """Publishes RViz markers for motion visualization.

    Markers are accumulated and the full list is re-published on every update,
    so TRANSIENT_LOCAL lets late-connecting RViz subscribers catch up.
    Call clear() between examples to reset the scene.

    RViz setup:
      Add display: MarkerArray → /constrained_planning_markers
    """

    def __init__(self, node, frame_id: str):
        self._node = node
        self._frame = frame_id
        self._pub_m = node.create_publisher(
            MarkerArray, '/constrained_planning_markers', _LATCHED
        )
        self._markers: list = []
        self._id = 0

    def _mk(self, mtype: int, ns: str, scale: Vector3, color: ColorRGBA, p: Pose) -> Marker:
        m = Marker()
        m.header.frame_id = self._frame
        m.header.stamp = self._node.get_clock().now().to_msg()
        m.ns = ns
        m.id = self._id
        self._id += 1
        m.type = mtype
        m.action = Marker.ADD
        m.pose = p
        m.scale = scale
        m.color = color
        return m

    def _emit(self, m: Marker):
        self._markers.append(m)
        arr = MarkerArray()
        arr.markers = list(self._markers)
        self._pub_m.publish(arr)

    def sphere(self, p: Pose, rgba: tuple, ns: str = 'points', diameter: float = 0.04):
        d = float(diameter)
        self._emit(self._mk(
            Marker.SPHERE, ns,
            Vector3(x=d, y=d, z=d),
            ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3]),
            p,
        ))

    def box(self, center: Pose, dims: tuple, rgba: tuple, ns: str = 'constraints'):
        """dims = (x, y, z) full extents in metres (matches SolidPrimitive.BOX convention)."""
        self._emit(self._mk(
            Marker.CUBE, ns,
            Vector3(x=float(dims[0]), y=float(dims[1]), z=float(dims[2])),
            ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3]),
            center,
        ))

    def trajectory_points(self, poses: list, rgba: tuple = (0.0, 0.6, 1.0, 0.8),
                          ns: str = 'trajectory', diameter: float = 0.008):
        """Show EE poses along a planned trajectory as small spheres.

        All spheres are added to the marker list and published in a single batch.
        Call robot.get_trajectory_ee_poses(traj) to get the poses list.
        """
        d = float(diameter)
        scale = Vector3(x=d, y=d, z=d)
        color = ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3])
        for p in poses:
            self._markers.append(self._mk(Marker.SPHERE, ns, scale, color, p))
        arr = MarkerArray()
        arr.markers = list(self._markers)
        self._pub_m.publish(arr)

    def clear(self):
        arr = MarkerArray()
        m = Marker()
        m.action = Marker.DELETEALL
        arr.markers.append(m)
        self._pub_m.publish(arr)
        self._markers.clear()
        self._id = 0
