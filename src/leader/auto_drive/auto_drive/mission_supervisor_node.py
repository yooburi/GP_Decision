import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from auto_drive.mission_supervisor_logic import MissionPolicy
from auto_drive.mission_supervisor_logic import MissionSupervisorCore


TRANSIENT_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class MissionSupervisorNode(Node):
    def __init__(self):
        super().__init__('mission_supervisor_node')

        self.throttle_from_planning_topic = self.declare_parameter(
            'throttle_from_planning_topic', '/throttle_from_planning'
        ).value
        self.throttle_cmd_topic = self.declare_parameter(
            'throttle_cmd_topic', '/throttle_cmd'
        ).value
        self.drive_context_topic = self.declare_parameter(
            'drive_context_topic', '/drive_context'
        ).value
        self.safety_stop_topic = self.declare_parameter(
            'safety_stop_topic', '/safety_stop'
        ).value
        self.manual_stop_topic = self.declare_parameter(
            'manual_stop_topic', '/manual_stop'
        ).value
        self.traffic_stop_topic = self.declare_parameter(
            'traffic_stop_topic', '/traffic_stop'
        ).value
        self.intersection_topic = self.declare_parameter(
            'intersection_topic', '/intersection'
        ).value
        self.roi_warning_topic = self.declare_parameter(
            'roi_warning_topic', '/roi_warning'
        ).value
        self.mission_state_topic = self.declare_parameter(
            'mission_state_topic', '/mission_state'
        ).value
        self.active_algorithm_topic = self.declare_parameter(
            'active_algorithm_topic', '/active_algorithm'
        ).value
        self.safety_status_topic = self.declare_parameter(
            'safety_status_topic', '/safety_status'
        ).value
        self.safety_active_topic = self.declare_parameter(
            'safety_active_topic', '/safety_active'
        ).value
        self.mission_marker_topic = self.declare_parameter(
            'mission_marker_topic', '/mission_state_marker'
        ).value
        self.mission_marker_frame_id = self.declare_parameter(
            'mission_marker_frame_id', 'leader/base_link'
        ).value
        self.mission_marker_text_z_m = float(
            self.declare_parameter('mission_marker_text_z_m', 2.1).value
        )
        self.mission_marker_text_scale = max(
            float(
                self.declare_parameter(
                    'mission_marker_text_scale', 0.42
                ).value
            ),
            0.05,
        )
        self.safety_marker_topic = self.declare_parameter(
            'safety_marker_topic', '/safety_preemption_markers'
        ).value
        self.safety_marker_frame_id = self.declare_parameter(
            'safety_marker_frame_id', 'leader/base_link'
        ).value
        self.safety_marker_radius_m = max(
            float(
                self.declare_parameter('safety_marker_radius_m', 2.4).value
            ),
            0.1,
        )
        self.safety_marker_height_m = max(
            float(
                self.declare_parameter(
                    'safety_marker_height_m', 0.18
                ).value
            ),
            0.01,
        )
        self.safety_marker_text_z_m = float(
            self.declare_parameter('safety_marker_text_z_m', 1.4).value
        )
        self.safety_marker_text_scale = max(
            float(
                self.declare_parameter(
                    'safety_marker_text_scale', 0.45
                ).value
            ),
            0.05,
        )
        self.command_timeout_sec = float(
            self.declare_parameter('command_timeout_sec', 0.5).value
        )
        self.release_hysteresis_sec = float(
            self.declare_parameter('release_hysteresis_sec', 0.5).value
        )
        self.publish_rate_hz = float(
            self.declare_parameter('publish_rate_hz', 20.0).value
        )
        self.default_mission = self.declare_parameter(
            'default_mission', 'highway'
        ).value

        highway_policy = MissionPolicy(
            throttle_scale=float(
                self.declare_parameter('highway_throttle_scale', 1.0).value
            ),
            throttle_limit=float(
                self.declare_parameter('highway_throttle_limit', 0.7).value
            ),
        )
        city_policy = MissionPolicy(
            throttle_scale=float(
                self.declare_parameter('city_throttle_scale', 0.75).value
            ),
            throttle_limit=float(
                self.declare_parameter('city_throttle_limit', 0.5).value
            ),
        )
        complex_policy = MissionPolicy(
            throttle_scale=float(
                self.declare_parameter('complex_throttle_scale', 0.6).value
            ),
            throttle_limit=float(
                self.declare_parameter('complex_throttle_limit', 0.4).value
            ),
        )

        self.core = MissionSupervisorCore(
            command_timeout_sec=self.command_timeout_sec,
            release_hysteresis_sec=self.release_hysteresis_sec,
            default_mission=self.default_mission,
            highway_policy=highway_policy,
            city_policy=city_policy,
            complex_policy=complex_policy,
        )

        self.last_logged_mission_state = None
        self.last_logged_safety_status = None
        self.last_logged_safety_active = None
        self.last_invalid_context_warn_ns = 0

        self.create_subscription(
            Float32,
            self.throttle_from_planning_topic,
            self.throttle_callback,
            10,
        )
        self.create_subscription(
            String,
            self.drive_context_topic,
            self.drive_context_callback,
            10,
        )
        self.create_subscription(
            Bool,
            self.safety_stop_topic,
            self.safety_stop_callback,
            10,
        )
        self.create_subscription(
            Bool,
            self.manual_stop_topic,
            self.manual_stop_callback,
            10,
        )
        self.create_subscription(
            Bool,
            self.traffic_stop_topic,
            self.traffic_stop_callback,
            10,
        )
        self.create_subscription(
            Bool,
            self.intersection_topic,
            self.intersection_callback,
            10,
        )
        self.create_subscription(
            Bool,
            self.roi_warning_topic,
            self.roi_warning_callback,
            10,
        )

        self.throttle_cmd_pub = self.create_publisher(
            Float32, self.throttle_cmd_topic, 10
        )
        self.mission_state_pub = self.create_publisher(
            String, self.mission_state_topic, 10
        )
        self.active_algorithm_pub = self.create_publisher(
            String, self.active_algorithm_topic, 10
        )
        self.safety_status_pub = self.create_publisher(
            String, self.safety_status_topic, 10
        )
        self.safety_active_pub = self.create_publisher(
            Bool, self.safety_active_topic, 10
        )
        self.mission_marker_pub = self.create_publisher(
            Marker, self.mission_marker_topic, TRANSIENT_QOS
        )
        self.safety_marker_pub = self.create_publisher(
            MarkerArray, self.safety_marker_topic, TRANSIENT_QOS
        )

        self.timer = self.create_timer(
            1.0 / max(self.publish_rate_hz, 1e-3), self.timer_callback
        )

        self.get_logger().info(
            'Mission supervisor ready: preemptive safety layer + '
            'mission-state throttle policies.'
        )

    def now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def throttle_callback(self, msg: Float32):
        self.core.set_planning_throttle(msg.data, self.now_sec())

    def drive_context_callback(self, msg: String):
        parsed_state = self.core.set_drive_context(msg.data)
        if parsed_state is None:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self.last_invalid_context_warn_ns > 2_000_000_000:
                self.get_logger().warn(
                    f'Ignoring unsupported drive context: {msg.data!r}'
                )
                self.last_invalid_context_warn_ns = now_ns

    def safety_stop_callback(self, msg: Bool):
        self.core.set_generic_safety_stop(msg.data)

    def manual_stop_callback(self, msg: Bool):
        self.core.set_manual_stop(msg.data)

    def traffic_stop_callback(self, msg: Bool):
        self.core.set_traffic_stop(msg.data)

    def intersection_callback(self, msg: Bool):
        self.core.set_intersection(msg.data)

    def roi_warning_callback(self, msg: Bool):
        self.core.set_roi_warning(msg.data)

    def timer_callback(self):
        snapshot = self.core.update(self.now_sec())
        self.log_state_changes(snapshot)
        self.publish_snapshot(snapshot)

    def log_state_changes(self, snapshot):
        if snapshot.mission_state != self.last_logged_mission_state:
            self.get_logger().info(
                f'Mission state -> {snapshot.mission_state.value}'
            )
            self.last_logged_mission_state = snapshot.mission_state

        if snapshot.safety_status != self.last_logged_safety_status:
            self.get_logger().info(
                f'Safety status -> {snapshot.safety_status.value}'
            )
            self.last_logged_safety_status = snapshot.safety_status

        if snapshot.safety_active != self.last_logged_safety_active:
            state_text = 'active' if snapshot.safety_active else 'released'
            self.get_logger().info(f'Safety preemption {state_text}.')
            self.last_logged_safety_active = snapshot.safety_active

    def publish_snapshot(self, snapshot):
        throttle_msg = Float32()
        throttle_msg.data = float(snapshot.output_throttle)
        self.throttle_cmd_pub.publish(throttle_msg)

        mission_msg = String()
        mission_msg.data = snapshot.mission_state.value
        self.mission_state_pub.publish(mission_msg)

        algorithm_msg = String()
        algorithm_msg.data = snapshot.active_algorithm.value
        self.active_algorithm_pub.publish(algorithm_msg)

        safety_status_msg = String()
        safety_status_msg.data = snapshot.safety_status.value
        self.safety_status_pub.publish(safety_status_msg)

        safety_active_msg = Bool()
        safety_active_msg.data = bool(snapshot.safety_active)
        self.safety_active_pub.publish(safety_active_msg)

        self.publish_mission_marker(snapshot)
        self.publish_safety_markers(snapshot)

    def publish_mission_marker(self, snapshot):
        marker = Marker()
        marker.header.stamp = Time().to_msg()
        marker.header.frame_id = self.mission_marker_frame_id
        marker.ns = 'mission_state_text'
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.frame_locked = True
        marker.pose.orientation.w = 1.0
        marker.pose.position.z = self.mission_marker_text_z_m
        marker.scale.z = self.mission_marker_text_scale
        color = self.mission_state_color(snapshot.mission_state.value)
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1.0
        marker.text = (
            f'MISSION: {snapshot.mission_state.value}\n'
            f'ALG: {snapshot.active_algorithm.value}'
        )
        self.mission_marker_pub.publish(marker)

    def publish_safety_markers(self, snapshot):
        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        clear = Marker()
        clear.header.stamp = stamp
        clear.header.frame_id = self.safety_marker_frame_id
        clear.action = Marker.DELETEALL
        marker_array.markers.append(clear)

        if snapshot.safety_active:
            marker_array.markers.append(
                self.make_safety_area_marker(stamp)
            )
            marker_array.markers.append(
                self.make_safety_text_marker(stamp, snapshot)
            )

        self.safety_marker_pub.publish(marker_array)

    def make_safety_area_marker(self, stamp):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.safety_marker_frame_id
        marker.ns = 'safety_preemption_area'
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.pose.position.z = self.safety_marker_height_m * 0.5
        marker.scale.x = self.safety_marker_radius_m * 2.0
        marker.scale.y = self.safety_marker_radius_m * 2.0
        marker.scale.z = self.safety_marker_height_m
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.38
        return marker

    def make_safety_text_marker(self, stamp, snapshot):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.safety_marker_frame_id
        marker.ns = 'safety_preemption_text'
        marker.id = 1
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.pose.position.z = self.safety_marker_text_z_m
        marker.scale.z = self.safety_marker_text_scale
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.text = (
            'SAFETY HOLD\n'
            f'{snapshot.safety_status.value}\n'
            f'throttle={snapshot.output_throttle:.2f}'
        )
        return marker

    @staticmethod
    def mission_state_color(mission_state):
        if mission_state == 'COMPLEX':
            return 1.0, 0.25, 0.9
        if mission_state == 'CITY':
            return 1.0, 0.85, 0.05
        if mission_state == 'HIGHWAY':
            return 0.1, 0.8, 1.0
        return 1.0, 1.0, 1.0


def main(args=None):
    rclpy.init(args=args)
    node = MissionSupervisorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
