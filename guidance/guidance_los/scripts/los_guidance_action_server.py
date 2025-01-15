#!/usr/bin/env python3

import threading

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
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

        # Initialize goal handle lock
        self._goal_lock = threading.Lock()

        # update rate parameter
        self.declare_parameter('update_rate', 10.0)
        update_rate = self.get_parameter('update_rate').value
        self.update_period = 1.0 / update_rate

        # Initialize filter status flag
        self.filter_initialized = False

        # Create callback groups for concurrent execution
        self.action_cb_group = ReentrantCallbackGroup()

        # Initialize parameters and topics
        self.declare_los_parameters_()
        self._declare_topic_parameters()
        self._initialize_publishers()
        self._initialize_subscribers()
        self._initialize_state()
        self._initialize_action_server()

        los_params = self.get_los_parameters_()
        filter_params = self.get_filter_parameters_()

        # Initialize guidance calculator with third-order filtering
        self.guidance_calculator = ThirdOrderLOSGuidance(los_params, filter_params)

        # self.desired_vel = 0.3

    def declare_los_parameters_(self):
        """Declare all LOS guidance parameters."""
        # Declare main LOS parameters
        self.declare_parameter('los_guidance.h_delta_min', 1.0)
        self.declare_parameter('los_guidance.h_delta_max', 5.0)
        self.declare_parameter('los_guidance.h_delta_factor', 3.0)
        self.declare_parameter('los_guidance.nominal_speed', 0.35)
        self.declare_parameter('los_guidance.min_speed', 0.1)
        self.declare_parameter('los_guidance.max_pitch_angle', 0.52359877559)
        self.declare_parameter('los_guidance.depth_gain', 0.5)

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
        """Declare parameters for all topics."""
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

        # QoS settings
        self.declare_parameter('qos.publisher_depth', 10)
        self.declare_parameter('qos.subscriber_depth', 10)

    def _initialize_publishers(self):
        """Initialize all publishers."""
        los_commands_topic = self.get_parameter('topics.publishers.los_commands').value
        self.get_logger().info(f"Publishing LOS commands to: {los_commands_topic}")
        self.guidance_cmd_pub = self.create_publisher(
            LOSGuidance, los_commands_topic, qos_profile=best_effort_qos
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

        return CancelResponse.ACCEPT

    def odom_callback(self, msg: Odometry):
        """Process odometry updates and trigger guidance calculations."""
        orientation_q = msg.pose.pose.orientation
        self.phi, self.pitch, yaw = self.guidance_calculator.quaternion_to_euler_angle(
            orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z
        )

        # Update vehicle state
        self.state.x = msg.pose.pose.position.x
        self.state.y = msg.pose.pose.position.y
        self.state.z = msg.pose.pose.position.z
        self.state.yaw = yaw
        self.state.pitch = self.pitch

        self.u = msg.twist.twist.linear.x
        self.v = msg.twist.twist.linear.y
        self.w = msg.twist.twist.linear.z

        # Initialize filter on first callback
        if not self.filter_initialized:
            self.guidance_calculator.reset_filter_state(self.state)
            self.filter_initialized = True

    def calculate_guidance(self) -> State:
        error = (
            self.state.as_pos_array()
            - self.waypoints[self.current_waypoint_index].as_pos_array()
        )
        _, crosstrack_y, crosstrack_z = self.rotation_yz @ error
        alpha_c = self.guidance_calculator.calculate_alpha_c(
            self.u, self.v, self.w, self.phi
        )
        beta_c = self.guidance_calculator.calculate_beta_c(
            self.u, self.v, self.w, self.phi, self.pitch, alpha_c
        )
        psi_d = self.guidance_calculator.compute_psi_d(
            self.waypoints[self.current_waypoint_index],
            self.waypoints[self.current_waypoint_index + 1],
            crosstrack_y,
            beta_c,
        )
        theta_d = self.guidance_calculator.compute_theta_d(
            self.waypoints[self.current_waypoint_index],
            self.waypoints[self.current_waypoint_index + 1],
            crosstrack_z,
            alpha_c,
        )

        yaw_error = self.guidance_calculator.ssa(psi_d - self.state.yaw)

        desired_surge = self.guidance_calculator.compute_desired_speed(
            yaw_error, self.norm
        )

        unfiltered_commands = State(surge_vel=desired_surge, pitch=theta_d, yaw=psi_d)
        filtered_commands = self.guidance_calculator.apply_reference_filter(
            unfiltered_commands
        )
        # return unfiltered_commands
        return filtered_commands

    def execute_callback(self, goal_handle: ServerGoalHandle):
        """Execute waypoint navigation action."""
        # Initialize navigation goal with lock
        with self._goal_lock:
            self.goal_handle = goal_handle
        self.current_waypoint_index = 0
        self.waypoints = [self.state]
        incoming_waypoints = goal_handle.request.waypoints
        for waypoint in incoming_waypoints:
            point_as_state = State(
                x=waypoint.pose.position.x,
                y=waypoint.pose.position.y,
                z=waypoint.pose.position.z,
            )
            self.waypoints.append(point_as_state)

        self.pi_h = self.guidance_calculator.compute_pi_h(
            self.waypoints[self.current_waypoint_index],
            self.waypoints[self.current_waypoint_index + 1],
        )
        self.pi_v = self.guidance_calculator.compute_pi_v(
            self.waypoints[self.current_waypoint_index],
            self.waypoints[self.current_waypoint_index + 1],
        )

        rotation_y = self.guidance_calculator.compute_rotation_y(self.pi_v)
        rotation_z = self.guidance_calculator.compute_rotation_z(self.pi_h)
        self.rotation_yz = rotation_y.T @ rotation_z.T

        # feedback = NavigateWaypoints.Feedback()
        result = NavigateWaypoints.Result()

        # Monitor navigation progress
        rate = self.create_rate(1.0 / self.update_period)

        self.get_logger().info('Executing goal')
        while rclpy.ok():
            if not goal_handle.is_active:
                # Preempted by another goal
                result.success = False
                self.guidance_calculator.reset_filter_state(self.state)
                return result

            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal canceled')
                goal_handle.canceled()
                self.goal_handle = None
                result.success = False
                return result

            self.norm = np.linalg.norm(
                self.state.as_pos_array()
                - self.waypoints[self.current_waypoint_index + 1].as_pos_array(),
                2,
            )
            if self.norm < 1.5:
                self.get_logger().info('Waypoint reached')
                self.current_waypoint_index += 1

                if self.current_waypoint_index >= len(self.waypoints) - 1:
                    self.get_logger().info('All waypoints reached!')
                    final_commands = State(
                        surge_vel=0.0, pitch=self.state.pitch, yaw=self.state.yaw
                    )
                    self.publish_guidance(final_commands)
                    result.success = True
                    self.goal_handle.succeed()
                    return result

                self.pi_h = self.guidance_calculator.compute_pi_h(
                    self.waypoints[self.current_waypoint_index],
                    self.waypoints[self.current_waypoint_index + 1],
                )
                self.pi_v = self.guidance_calculator.compute_pi_v(
                    self.waypoints[self.current_waypoint_index],
                    self.waypoints[self.current_waypoint_index + 1],
                )

                rotation_y = self.guidance_calculator.compute_rotation_y(self.pi_v)
                rotation_z = self.guidance_calculator.compute_rotation_z(self.pi_h)
                self.rotation_yz = rotation_y.T @ rotation_z.T

            commands = self.calculate_guidance()
            self.publish_guidance(commands)

            rate.sleep()

    def publish_guidance(self, commands: State):
        """Publish the commanded surge velocity, pitch angle, and yaw angle."""
        msg = LOSGuidance()
        msg.surge = commands.surge_vel
        msg.pitch = commands.pitch
        msg.yaw = commands.yaw
        self.guidance_cmd_pub.publish(msg)


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
