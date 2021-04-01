#!/usr/bin/env python
# coding: UTF-8
"""

Node som forenkler tilganger til controlleren. 
Den skal ta av seg bytting mellom controller moduser, 
slik at koden i state machinene kan bli enklere. 

Noden skal ha en action-server som tar inn en ny action (ligner på
move_base) som skal inneholde: 
- Target pose og 
- String for kontroller som skal benyttes

Noden sender så target posen videre som en ny action
til den valgte kontrolleren. Resultat og feedback fra endelig kontroller
sendes videre igjennom noden. 

"""

import rospy
import actionlib
import time

from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty

from vortex_msgs.msg import (
    MoveAction,
    LosPathFollowingAction,
    LosPathFollowingGoal,
    ControlModeAction,
    SetVelocityAction,
)
from vortex_msgs.srv import ControlMode, SetVelocity, SetVelocityRequest

# ENUM for controller mode
OPEN_LOOP = 0
POSE_HOLD = 1
HEADING_HOLD = 2
DEPTH_HEADING_HOLD = 3
DEPTH_HOLD = 4
POSE_HEADING_HOLD = 5
CONTROL_MODE_END = 6


def change_control_mode(requested_mode):
    """
    Change the controller mode of the DP controller.

    Although this is technically breaking the design
    idea that FSM and controller should not talk to
    eachother, it makes sense to keep it for now, seeing
    as some guidance systems may rely on one of the control
    modes for the controller.

    In the future, it may be moved to the DP guidance node,
    if it makes sense to.

    """

    rospy.wait_for_service("/controller/controlmode_service")  # From controller_ros.cpp

    try:
        control_mode = rospy.ServiceProxy(
            "/controller/controlmode_service", ControlMode
        )
        response = control_mode(requested_mode)
        return response.result

    except rospy.ServiceException as e:
        rospy.logerr("guidance_interface could not change control mode")
        print("Service call failed: %s" % e)


class GuidanceInterface_old:
    def __init__(self):
        """
        Define constants used in the guidance systems, create the
        move action server that the fsm uses to communicate with
        this node,  and connect to the actions servers in the guidance
        systems.
        """

        self.transit_speed = rospy.get_param("~transit_speed", 0.3)
        self.sphere_of_acceptance = rospy.get_param("~sphere_of_acceptance", 0.5)
        self.timeout = rospy.get_param("~guidance_interface_timeout", 90)

        self.action_server = actionlib.SimpleActionServer(
            "move", MoveAction, self.move_cb, auto_start=False
        )
        self.action_server.start()

        self.dp_client = actionlib.SimpleActionClient(
            "dp_action_server", MoveBaseAction
        )
        self.los_client = actionlib.SimpleActionClient(
            "los_action_server", LosPathFollowingAction
        )

        self.joystick_sub = rospy.Subscriber(
            "/joystick/joy", Joy, self.joystick_cb, queue_size=1
        )
        self.joystick_pub = rospy.Publisher(
            "/guidance/joystick_data", Joy, queue_size=1
        )

        rospy.loginfo("Guidance interface is up and running")

    def move_cb(self, move_goal):
        """
        Converts move_goal into the proper goal type for the desired guidance
        system and engages the corresponding guidance node through the
        action servers.

        Aborts action if it is not completed within self.timeout seconds
        """

        # Talk to the dp_guidance node...
        if move_goal.guidance_type == "PositionHold":
            rospy.loginfo("move_cb -> PositionHold. Changing control mode...")
            change_control_mode(POSE_HEADING_HOLD)

            dp_goal = MoveBaseGoal()
            dp_goal.target_pose.pose = move_goal.target_pose

            self.dp_client.send_goal(dp_goal, done_cb=self.done_cb, feedback_cb=None)

            if not self.dp_client.wait_for_result(timeout=rospy.Duration(self.timeout)):
                self.action_server.set_aborted()
                rospy.loginfo("DP guidance aborted action due to timeout")

        # Talk to the los_guidance node...
        elif move_goal.guidance_type == "LOS":
            rospy.loginfo("move_cb -> LOS. Changing control mode...")
            change_control_mode(OPEN_LOOP)

            los_goal = LosPathFollowingGoal()
            los_goal.next_waypoint = move_goal.target_pose.position
            los_goal.forward_speed.linear.x = self.transit_speed
            los_goal.sphereOfAcceptance = self.sphere_of_acceptance
            los_goal.desired_depth.z = move_goal.target_pose.position.z

            self.los_client.send_goal(los_goal, done_cb=self.done_cb, feedback_cb=None)

            if not self.los_client.wait_for_result(
                timeout=rospy.Duration(self.timeout)
            ):
                self.action_server.set_aborted()
                rospy.loginfo("LOS guidance aborted action due to timeout")

        else:
            rospy.logerr("Unknown guidace type sent to guidance_interface")
            self.action_server.set_aborted()

    def done_cb(self, state, result):
        """
        Set the outcome of the action depending on
        the returning result.
        """

        if state == GoalStatus.SUCCEEDED:
            self.action_server.set_succeeded()

        elif state == GoalStatus.PREEMPTED:
            self.action_server.set_preempted()

        else:
            self.action_server.set_aborted()


class JoyGuidance:
    def __init__(self, guidance_interface) -> None:
        self.guidance_interface = guidance_interface

        # get params

        # set up servers and clients
        self.joystick_controlmode_server = actionlib.SimpleActionServer(
            "control_mode_server",
            ControlModeAction,
            self.joystick_control_mode_cb,
            auto_start=False,
        )

        # wait for external services and start
        rospy.wait_for_service("/controller/controlmode_service")
        self.joystick_controlmode_server.start()

    def joystick_control_mode_cb(self, control_mode):
        change_control_mode(control_mode.controlModeIndex)
        time.sleep(0.25)  # Avoid aggressive switching

    def stop(self):
        pass


class VelocityGuidance:
    def __init__(self, guidance_interface):
        self.guidance_interface = guidance_interface

        # params
        set_velocity_service_name = "/vel_guidance/set_velocity"
        stop_guidance_service_name = "/vel_guidance/stop_guidance"
        action_server_name = "vel_server"

        # set up servers and clients
        rospy.wait_for_service(self.set_velocity_service)
        self.set_velocity_service = rospy.ServiceProxy(
            set_velocity_service_name, SetVelocity
        )
        rospy.wait_for_service(self.stop_guidance_service)
        self.stop_guidance_service = rospy.ServiceProxy(
            stop_guidance_service_name, Empty
        )
        self.action_server = actionlib.SimpleActionServer(
            action_server_name, MoveBaseAction, self.dpCallback, auto_start=False
        )
        self.action_server.start()

    def callback(self, set_velocity_goal):
        self.guidance_interface.stopAllGuidance()

        request = SetVelocityRequest()
        request.desired_velocity = set_velocity_goal.desired_velocity

        try:
            self.set_velocity_service(request)
        except rospy.ServiceException as exc:
            rospy.logerr("Set velocity service did not process request: " + str(exc))

    def stop(self):
        try:
            self.stop_guidance_service()
        except rospy.ServiceException as exc:
            rospy.logerr(
                "Stop velocity guidance service did not process request: " + str(exc)
            )


class DpGuidance:
    def __init__(self, guidance_interface):
        self.guidance_interface = guidance_interface

        # params
        dp_guidance_action_server = "dp_action_server"
        guidance_interface_dp_action_server = "dp_server"

        # set up servers and clients
        self.dp_move_client = actionlib.SimpleActionClient(
            dp_guidance_action_server, MoveBaseAction
        )
        self.dp_server = actionlib.SimpleActionServer(
            guidance_interface_dp_action_server,
            MoveBaseAction,
            self.dpCallback,
            auto_start=False,
        )
        self.dp_server.start()

    def callback(self, goal):
        self.guidance_interface.stop_all_guidance()

    def stop(self):
        pass


class LosGuidance:
    def __init__(self, guidance_interface):
        self.guidance_interface = guidance_interface

        # params
        self.los_transit_speed = rospy.get_param("guidance/LOS/transit_speed", 0.3)
        self.los_sphere_of_acceptance = rospy.get_param(
            "guidance/LOS/sphere_of_acceptance", 0.5
        )
        self.los_timeout = rospy.get_param(
            "guidance/LOS/guidance_interface_timeout", 90
        )

        los_guidance_action_server = "los_action_server"
        guidance_interface_los_action_server = "los_server"

        # set up servers and clients
        self.action_client = actionlib.SimpleActionClient(
            los_guidance_action_server, LosPathFollowingAction
        )
        self.action_server = actionlib.SimpleActionServer(
            guidance_interface_los_action_server,
            LosPathFollowingAction,
            self.los_callback,
            auto_start=False,
        )
        self.action_server.start()

    def los_callback(self, los_goal):
        self.guidance_interface.stop_all_guidance()

        self.action_client.send_goal(
            los_goal, done_cb=self.guidance_finished_cb, feedback_cb=None
        )

        if not self.action_client.wait_for_result(timeout=rospy.Duration(self.timeout)):
            self.action_server.set_aborted()
            rospy.loginfo("LOS guidance aborted action due to timeout")

    def guidance_finished_cb(self, state, result):
        """
        Set the outcome of the action depending on
        the returning result.
        """

        if state == GoalStatus.SUCCEEDED:
            self.action_server.set_succeeded()

        elif state == GoalStatus.PREEMPTED:
            self.action_server.set_preempted()

        else:
            self.action_server.set_aborted()

    def stop(self):
        self.action_client.cancel_all_goals()


class GuidanceInterface:
    def __init__(self):
        self.vel_guidance = VelocityGuidance(self)
        self.dp_guidance = DpGuidance(self)
        self.los_guidance = LosGuidance(self)
        self.joy_guidance = JoyGuidance(self)

    def stop_all_guidance(self):
        self.joy_guidance.stop()
        self.vel_guidance.stop()
        self.dp_guidance.stop()
        self.los_guidance.stop()


if __name__ == "__main__":
    rospy.init_node("interface")
    server = GuidanceInterface()
    rospy.spin()
