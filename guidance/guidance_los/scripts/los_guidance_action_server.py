#!/usr/bin/env python3

import threading

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from guidance_los.los_guidance_algorithm import (
    FilterParameters,
    LOSParameters,
    State,
    ThirdOrderLOSGuidance,
)
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer
from rclpy.action.server import CancelResponse, GoalResponse, ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from vortex_msgs.action import NavigateWaypoints
from vortex_msgs.msg import LOSGuidance

best_effort_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class LOSActionServer(Node):
    """ROS2 Action Server node for waypoint navigation using LOS guidance."""

    def __init__(self):
        super().__init__('los_guidance_node')

        self._goal_lock = threading.Lock()
        self.declare_parameter('update_rate', 10.0)
        update_rate = self.get_parameter('update_rate').value
        self.update_period = 1.0 / update_rate
        self.filter_initialized = False

        self.declare_parameter('debug_mode', False)
        self.debug_mode = self.get_parameter('debug_mode').value

        self.action_cb_group = ReentrantCallbackGroup()

        self.declare_los_parameters_()
        self._declare_topic_parameters()
        self._initialize_publishers()
        self._initialize_subscribers()
        self._initialize_state()
        self._initialize_action_server()

        los_params = self.get_los_parameters_()
        filter_params = self.get_filter_parameters_()

        # Initialize guidance calculator
        self.guidance_calculator = ThirdOrderLOSGuidance(los_params, filter_params)

    def declare_los_parameters_(self):
        """Declare all LOS guidance parameters."""
        # Declare main LOS parameters
        self.declare_parameter('los_guidance.h_delta_min', 1.0)
        self.declare_parameter('los_guidance.h_delta_max', 5.0)
        self.declare_parameter('los_guidance.h_delta_factor', 3.0)
        self.declare_parameter('los_guidance.nominal_speed', 0.35)
        self.declare_parameter('los_guidance.min_speed', 0.1)
        self.declare_parameter('los_guidance.max_pitch_angle', 1.047)
        self.declare_parameter('los_guidance.depth_gain', 10.0)
        # self.declare_parameter('depth_integral_gain', 0.5)

        # Declare filter parameters
        self.declare_parameter('los_guidance.filter.omega_diag', [2.5, 2.5, 2.5])
        self.declare_parameter('los_guidance.filter.zeta_diag', [0.7, 0.7, 0.7])

    def get_los_parameters_(self) -> LOSParameters:
        los_params = LOSParameters()

        los_params.lookahead_distance_min = self.get_parameter(
            'los_guidance.h_delta_min'
        ).value
        los_params.lookahead_distance_max = self.get_parameter(
            'los_guidance.h_delta_max'
        ).value
        los_params.lookahead_distance_factor = self.get_parameter(
            'los_guidance.h_delta_factor'
        ).value
        los_params.nominal_speed = self.get_parameter(
            'los_guidance.nominal_speed'
        ).value
        los_params.min_speed = self.get_parameter('los_guidance.min_speed').value
        los_params.max_pitch_angle = self.get_parameter(
            'los_guidance.max_pitch_angle'
        ).value
        los_params.depth_gain = self.get_parameter('los_guidance.depth_gain').value
        return los_params

    def get_filter_parameters_(self) -> FilterParameters:
        filter_params = FilterParameters()
        filter_params.omega_diag = self.get_parameter(
            'los_guidance.filter.omega_diag'
        ).value
        filter_params.zeta_diag = self.get_parameter(
            'los_guidance.filter.zeta_diag'
        ).value
        return filter_params

    def _declare_topic_parameters(self):
        # Publishers
        self.declare_parameter('topics.publishers.los_commands', '/guidance/los')
        self.declare_parameter(
            'topics.publishers.debug.reference', '/guidance/debug/reference'
        )
        self.declare_parameter(
            'topics.publishers.debug.errors', '/guidance/debug/errors'
        )
        self.declare_parameter('topics.publishers.debug.logs', '/guidance/debug/logs')

        # Subscribers
        self.declare_parameter('topics.subscribers.odometry', '/orca/odom')

    def _initialize_publishers(self):
        """Initialize all publishers."""
        pub_qos_depth = 10

        # Main guidance command publisher
        los_commands_topic = self.get_parameter('topics.publishers.los_commands').value

        self.get_logger().info(f"Publishing LOS commands to: {los_commands_topic}")

        self.guidance_cmd_pub = self.create_publisher(
            LOSGuidance, los_commands_topic, qos_profile=best_effort_qos
        )

        # Debug publishers
        if self.debug_mode:
            reference_topic = self.get_parameter(
                'topics.publishers.debug.reference'
            ).value

            errors_topic = self.get_parameter('topics.publishers.debug.errors').value

            logs_topic = self.get_parameter('topics.publishers.debug.logs').value

            self.guidance_ref_pub = self.create_publisher(
                PoseStamped, reference_topic, pub_qos_depth
            )
            self.guidance_error_pub = self.create_publisher(
                Vector3Stamped, errors_topic, pub_qos_depth
            )
            self.log_publisher = self.create_publisher(
                String, logs_topic, pub_qos_depth
            )

    def _initialize_subscribers(self):
        """Initialize subscribers."""
        odom_topic = self.get_parameter('topics.subscribers.odometry').value

        self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            qos_profile=best_effort_qos,
        )

    def _initialize_state(self):
        """Initialize state and navigation variables."""
        self.state: State = State()
        self.waypoints: list[PoseStamped] = []
        self.current_waypoint_index: int = 0
        self.goal_handle: ServerGoalHandle = None
        self.update_period = 0.1

    def _initialize_action_server(self):
        """Initialize the navigation action server."""
        self._action_server = ActionServer(
            self,
            NavigateWaypoints,
            'navigate_waypoints',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.action_cb_group,
        )

    def goal_callback(self, goal_request: NavigateWaypoints.Goal):
        """Handle new goal requests with preemption."""
        self.get_logger().info("Received new goal request")

        # Validate waypoints exist
        if not goal_request.waypoints:
            self.get_logger().warn("No waypoints in request, rejecting")
            return GoalResponse.REJECT

        with self._goal_lock:
            # If there's an active goal, preempt it
            if self.goal_handle and self.goal_handle.is_active:
                self.get_logger().info("Preempting current goal")
                self.goal_handle.abort()

            return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle: ServerGoalHandle):
        """Handle cancellation requests."""
        self.get_logger().info("Received cancel request")

        # with self._goal_lock:
        #     if self.goal_handle == goal_handle:
        #         # Reset navigation state
        #         self.waypoints = []
        #         self.current_waypoint_index = 0
        #         self.goal_handle = None

        #         # Reset guidance state if needed
        #         initial_commands = np.array([0.0, self.state.pitch, self.state.yaw])
        #         self.guidance_calculator.reset_filter_state(self.state)

        #         self.publish_log("Navigation canceled and state reset")

        return CancelResponse.ACCEPT

    def publish_log(self, message: str):
        """Publish debug log message to ROS topic and node logger (debug mode only)."""
        if not self.debug_mode:
            return

        msg = String()
        msg.data = message
        self.get_logger().info(message)

    def odom_callback(self, msg: Odometry):
        """Process odometry updates and trigger guidance calculations."""
        orientation_q = msg.pose.pose.orientation
        roll, pitch, yaw = self.guidance_calculator.quaternion_to_euler_angle(
            orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z
        )

        # Update vehicle state
        self.state.x = msg.pose.pose.position.x
        self.state.y = msg.pose.pose.position.y
        self.state.z = msg.pose.pose.position.z
        self.state.yaw = yaw
        self.state.pitch = pitch

        # Initialize filter on first callback
        if not self.filter_initialized:
            self.guidance_calculator.reset_filter_state(self.state)
            self.filter_initialized = True

        if not self.debug_mode:
            return

        # Log current position for debugging
        self.publish_log(
            f"Position: x={self.state.x:.3f}, "
            f"y={self.state.y:.3f}, "
            f"z={self.state.z:.3f}"
        )
        self.publish_log(
            f"Orientation: roll={roll:.3f}, pitch={pitch:.3f}, yaw={yaw:.3f}"
        )

    def process_guidance(self):
        """Process guidance calculations and publish control commands."""
        if self.current_waypoint_index >= len(self.waypoints):
            return

        # Extract position from PoseStamped waypoint
        current_waypoint = self.waypoints[self.current_waypoint_index]
        target_point = State(
            x=current_waypoint.pose.position.x,
            y=current_waypoint.pose.position.y,
            z=current_waypoint.pose.position.z,
        )
        # Compute guidance commands using current state
        commands = self.guidance_calculator.compute_guidance(self.state, target_point)

        # Publish commands and monitoring data
        self.publish_guidance(commands)
        # self.publish_errors(target_point, depth_error)

        # Log guidance status
        # self.publish_log(
        #     f"Guidance Status: surge={commands.surge_vel:.2f}, "
        #     f"pitch={commands[1]:.2f}, yaw={commands[2]:.2f}"
        # )
        # self.publish_log(f"Distance to target: {distance:.2f}")

        # Check if waypoint is reached (0.5m threshold)
        if self.guidance_calculator.horizontal_distance < 0.1:
            self.publish_log(f'Reached waypoint {self.current_waypoint_index}')

            self.guidance_calculator.reset_filter_state(self.state)

            self.current_waypoint_index += 1

            if self.current_waypoint_index >= len(self.waypoints):
                if self.goal_handle and self.goal_handle.is_active:
                    self.goal_handle.succeed()
                    self.goal_handle = None
                return

        # Publish progress feedback
        if self.goal_handle and self.goal_handle.is_active:
            feedback_msg = NavigateWaypoints.Feedback()
            feedback_msg.current_waypoint_index = float(self.current_waypoint_index)
            self.goal_handle.publish_feedback(feedback_msg)

    def execute_callback(self, goal_handle: ServerGoalHandle):
        """Execute waypoint navigation action."""
        self.publish_log('Executing waypoint navigation...')

        # Initialize navigation goal with lock
        with self._goal_lock:
            self.goal_handle = goal_handle
        self.current_waypoint_index = 0
        self.waypoints = goal_handle.request.waypoints

        feedback = NavigateWaypoints.Feedback()
        result = NavigateWaypoints.Result()

        self.publish_log(f'Received {len(self.waypoints)} waypoints')

        rate = self.create_rate(1.0 / self.update_period)

        self.get_logger().info('Executing goal')
        while rclpy.ok():
            if not goal_handle.is_active:
                # Preempted by another goal
                result.success = False
                self.guidance_calculator.reset_filter_state(self.state)
                return result

            if goal_handle.is_cancel_requested:
                self.publish_log('Goal canceled')
                goal_handle.canceled()
                self.goal_handle = None
                return NavigateWaypoints.Result(success=False)

            # process guiance
            if len(self.waypoints) > 0:
                self.process_guidance()

            if self.current_waypoint_index >= len(self.waypoints):
                self.publish_log('All waypoints reached')
                return NavigateWaypoints.Result(success=True)

            rate.sleep()

    def publish_guidance(self, commands: State):
        """Publish the commanded surge velocity, pitch angle, and yaw angle."""
        msg = LOSGuidance()
        msg.surge = commands.surge_vel
        msg.pitch = commands.pitch
        msg.yaw = commands.yaw
        self.guidance_cmd_pub.publish(msg)

    def publish_errors(self, target_point: np.ndarray, depth_error: float):
        """Publish position errors for monitoring (if debug mode is enabled)."""
        if not self.debug_mode:
            return

        msg = Vector3Stamped()
        msg.header.frame_id = "odom"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.vector.x = target_point[0] - self.state.x
        msg.vector.y = target_point[1] - self.state.y
        msg.vector.z = depth_error
        self.guidance_error_pub.publish(msg)


def main(args=None):
    """Main function to initialize and run the guidance node."""
    rclpy.init(args=args)
    action_server = LOSActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(action_server)

    try:
        executor.spin()
    except Exception as e:
        action_server.get_logger().error(f'Error: {str(e)}')
    finally:
        action_server.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
