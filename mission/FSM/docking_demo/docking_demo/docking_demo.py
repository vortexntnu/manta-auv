import time

import rclpy

# from action_tutorials_interfaces.action import Fibonacci
import rclpy.publisher
from go_to_dock_action.action import FindDock, GoToDock
from yasmin import Blackboard, StateMachine
from yasmin_ros import ActionState
from yasmin_ros.basic_outcomes import ABORT, SUCCEED
from yasmin_viewer import YasminViewerPub


class FindDockingStationState(ActionState):
    """
    The state to find the docking station. Using rectangle search pattern to find the docking station.
    """

    def __init__(self) -> None:
        """
        Initialize the state, and using ActionState from YASMIN.
        """
        super().__init__(FindDock, "/find_dock", self.create_goal_handler, None, self.response_handler, self.print_feedback)

    def create_goal_handler(self, blackboard: Blackboard) -> FindDock.Goal:
        """
        The goal handler to create the goal for the action. For this state, the goal is true or false depending on if the docking station is found.
        """

        goal = FindDock.Goal()
        goal.found = True
        return goal

    def response_handler(self, blackboard: Blackboard, response: FindDock.Result) -> str:
        """
        The response handler to handle the response from the action. For this state, the response is true or false depending on if the docking station is found.
        """
        blackboard["docking_station_location"] = response.docking_station_location
        # implement error handling
        return SUCCEED

    def print_feedback(self, blackboard: Blackboard, feedback: FindDock.Feedback) -> None:
        """
        Handles the feedback from the action. For this state, the feedback is how far it is in the search pattern.
        """
        blackboard["num_checked_waypoints"] = feedback.num_checked_waypoints
        print(f"Received feedback: {feedback.num_checked_waypoints}")


class GoToDockState(ActionState):
    """
    The state to go to the docking station. Using the waypoint action to go to the docking station.
    """

    def __init__(self) -> None:
        """
        Initialize the state, and using ActionState from YASMIN.
        """
        super().__init__(GoToDock, "/go_to_dock", self.create_goal_handler, None, self.response_handler, self.print_feedback)

    def create_goal_handler(self, blackboard: Blackboard) -> GoToDock.Goal:
        """
        The goal handler to create the goal for the action. For this state, the goal is the waypoint to the docking station.
        """
        goal = GoToDock.Goal()
        goal.docking_position = blackboard["docking_station_location"]
        return goal

    def response_handler(self, blackboard: Blackboard, response: GoToDock.Result) -> str:
        """
        Response handler to handle the response from the action. For this state, the response is true or false depending on if the auv is at the docking station.
        """
        blackboard["has_gone_to_docking_station"] = response.success
        if blackboard["has_gone_to_docking_station"]:
            return SUCCEED
        elif not blackboard["has_gone_to_docking_station"]:
            return ABORT

    def print_feedback(self, blackboard: Blackboard, feedback: GoToDock.Feedback) -> None:
        """
        Handles the feedback from the action. For this state, the feedback is the distance to the docking station.
        """
        print(f"Received feedback: {feedback.distance_to_dock}")


class DockState(ActionState):
    """
    The state to dock the auv to the docking station. Uses the DP controller.
    """

    def __init__(self) -> None:
        """
        Initialize the state, and using ActionState from YASMIN.
        """
        super().__init__(GoToDock, "/dock", self.create_goal_handler, None, self.response_handler, self.print_feedback)

    def create_goal_handler(self, blackboard: Blackboard) -> GoToDock.Goal:
        """
        The goal handler to create the goal for the action. For this state, the goal is true or false depending on if the auv is docked.
        """
        goal = GoToDock.Goal()
        goal.docking_station_location = blackboard["docking_station_location"]
        return goal

    def response_handler(self, blackboard: Blackboard, response: GoToDock.Result) -> str:
        """
        The response handler to handle the response from the action. For this state, the response is true or false depending on if the auv is docked.
        """
        blackboard["has_docked"] = response.success
        if blackboard["has_docked"]:
            return SUCCEED
        elif not blackboard["has_docked"]:
            return ABORT

    def print_feedback(self, blackboard: Blackboard, feedback: GoToDock.Feedback) -> None:
        """
        Handles the feedback from the action. For this state, the feedback comes from the DP controller.
        """
        print(f"Received feedback: {feedback.distance_to_dock}")


class DockedState(ActionState):
    """
    The state when the AUV is docked. The state will wait for the mission to be aborted or finished.
    """

    def __init__(self) -> None:
        """
        Initialize the state, and using ActionState from YASMIN."""
        super().__init__(GoToDock, "/waypoint", self.create_goal_handler, None, self.response_handler, self.print_feedback)

    def create_goal_handler(self, blackboard: Blackboard) -> GoToDock.Goal:
        """
        The goal handler to create the goal for the action. For this state, the goal is true or false depending on if the mission is aborted or finished.
        """
        goal = GoToDock.Goal()
        goal.order = blackboard["waypoint_res"]
        return goal

    def response_handler(self, blackboard: Blackboard, response: GoToDock.Result) -> str:
        """
        The response handler to handle the response from the action. For this state, the response is true or false depending on if the mission is aborted
        """
        blackboard["waypoint_res"] = response.sequence
        return SUCCEED

    def print_feedback(self, blackboard: Blackboard, feedback: GoToDock.Feedback) -> None:
        """
        Handles the feedback from the action. For this state, the feedback is idle if it is idle."""
        print(f"Received feedback: {list(feedback.partial_sequence)}")


class ReturnHomeState(ActionState):
    """ "
    The state to return home. Using the waypoint action to go back to the home position."""

    def __init__(self) -> None:
        """
        Initialize the state, and using ActionState from YASMIN."""
        super().__init__(GoToDock, "/waypoint", self.create_goal_handler, None, self.response_handler, self.print_feedback)

    def create_goal_handler(self, blackboard: Blackboard) -> GoToDock.Goal:
        """
        The goal handler to create the goal for the action. For this state, the goal is the waypoint to the home position."""
        goal = GoToDock.Goal()
        goal.order = blackboard["waypoint_res"]
        return goal

    def response_handler(self, blackboard: Blackboard, response: GoToDock.Result) -> str:
        """
        The response handler to handle the response from the action. For this state, the response is true or false depending on if the auv is at the home position.
        """
        blackboard["waypoint_res"] = response.sequence
        return SUCCEED

    def print_feedback(self, blackboard: Blackboard, feedback: GoToDock.Feedback) -> None:
        """
        Handles the feedback from the action. For this state, the feedback is the distance to the home position."""
        print(f"Received feedback: {list(feedback.partial_sequence)}")


class AbortState(ActionState):
    """
    The state to abort the mission. When the mission is aborted, the auv will return home."""

    def __init__(self) -> None:
        """
        Initialize the state, and using ActionState from YASMIN."""
        super().__init__(GoToDock, "/waypoint", self.create_goal_handler, None, self.response_handler, self.print_feedback)

    def create_goal_handler(self, blackboard: Blackboard) -> GoToDock.Goal:
        """
        The goal handler to create the goal for the action. For this state, the goal is true or false depending on if the mission is aborted."""
        goal = GoToDock.Goal()
        goal.order = blackboard["waypoint_res"]
        return goal

    def response_handler(self, blackboard: Blackboard, response: GoToDock.Result) -> str:
        """The response handler to handle the response from the action. For this state, the response is true or false depending on if the mission is aborted."""
        blackboard["waypoint_res"] = response.sequence
        return SUCCEED

    def print_feedback(self, blackboard: Blackboard, feedback: GoToDock.Feedback) -> None:
        """
        Handles the feedback from the action. For this state, the feedback is ABORT!!!."""
        print(f"Received feedback: {list(feedback.partial_sequence)}")


class ErrorState(ActionState):
    """
    State if an error occurs. This state will stop the mission."""

    def __init__(self) -> None:
        """
        Initialize the state, and using ActionState from YASMIN."""
        super().__init__(GoToDock, "/waypoint", self.create_goal_handler, None, self.response_handler, self.print_feedback)

    def create_goal_handler(self, blackboard: Blackboard) -> GoToDock.Goal:
        """
        The goal handler to create the goal for the action. For this state, the goal is to shut down or save or something."""
        goal = GoToDock.Goal()
        goal.order = blackboard["waypoint_res"]
        return goal

    def response_handler(self, blackboard: Blackboard, response: GoToDock.Result) -> str:
        """
        The response handler to handle the response from the action. For this state, the response is the error."""
        blackboard["waypoint_res"] = response.sequence
        return SUCCEED

    def print_feedback(self, blackboard: Blackboard, feedback: GoToDock.Feedback) -> None:
        """
        Handles the feedback from the action. For this state, the feedback is the error."""
        print(f"Received feedback: {list(feedback.partial_sequence)}")


def main() -> None:
    """
    Main function of the state machine.
    """
    print("yasmin_docking_fsm_demo")

    rclpy.init()

    # Create FSM with defined outcomes
    sm = StateMachine(outcomes=["error", "finished", "aborted"])

    # Create and initialize the blackboard
    blackboard = Blackboard()
    blackboard.distance = 10
    blackboard["waypoint_res"] = [0.0, 0.0, 0.0]
    blackboard["dock_pos"] = [5, 5, 10]
    blackboard["start_pos"] = [0, 0, 0]
    blackboard["Pool_dimensions"] = [30, 12, 10]

    # Adding states with transitions
    sm.add_state("find_dock", FindDockingStationState(), transitions={SUCCEED: "go_to_dock", ABORT: "abort_mission"})
    sm.add_state("go_to_dock", GoToDockState(), transitions={SUCCEED: "dock", ABORT: "abort_mission"})
    sm.add_state("dock", DockState(), transitions={SUCCEED: "docked", ABORT: "abort_mission"})
    sm.add_state("docked", DockedState(), transitions={ABORT: "abort_mission", SUCCEED: "return_home"})
    sm.add_state("return_home", ReturnHomeState(), transitions={SUCCEED: "finished", ABORT: "abort_mission"})
    sm.add_state("abort_mission", AbortState(), transitions={SUCCEED: "find_dock", ABORT: "aborted"})
    sm.add_state("error_state", ErrorState(), transitions={ABORT: "error"})

    # Set the initial state
    sm.set_start_state("find_dock")

    # Create a viewer to visualize the FSM (ensure YasminViewerPub is correctly set up)
    YasminViewerPub("Docking State Machine", sm)

    # Run the state machine
    outcome = sm(blackboard)
    print("Outcome: ", outcome)

    # Shutdown ROS2
    rclpy.shutdown()


if __name__ == "__main__":
    main()
