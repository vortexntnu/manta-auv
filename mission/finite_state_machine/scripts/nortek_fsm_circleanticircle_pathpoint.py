#!/usr/bin/python3

import rospy
from smach import StateMachine
from smach_ros import IntrospectionServer

# from nortek_circle import Circle
from nortek_anticircle import Circle
from nortek_reach_depth import ReachDepth
# from search.forward_sweep import ForwardSweepSearch


def main():
    rospy.init_node("nortek_fsm")

    rospy.sleep(rospy.get_param("/fsm/time_to_launch"))

    #rospy.wait_for_service(
    #    "send_positions"
    #) 

    nortek_fsm = StateMachine(
        outcomes=["preempted", "succeeded", "aborted"])


    with nortek_fsm:

        #StateMachine.add(
        #    "REACH_DEPTH", ReachDepth(), transitions={"succeeded": "CIRCLE"}
        #)

        StateMachine.add("CIRCLE", Circle(), transitions={"succeeded": "ANTICIRCLE"})
        StateMachine.add("ANTICIRCLE", Circle(anti_circle=True), transitions={"succeeded": "CIRCLE_POINT"})

        StateMachine.add("CIRCLE_POINT", Circle(is_path_dependent=False), transitions={"succeeded": "ANTICIRCLE_POINT"})
        StateMachine.add("ANTICIRCLE_POINT", Circle(anti_circle=True, is_path_dependent=False))

    intro_server = IntrospectionServer(
        str(rospy.get_name()), nortek_fsm, "/SM_ROOT"
    )
    intro_server.start()

    try:
        nortek_fsm.execute()
        intro_server.stop()

    except Exception as e:
        rospy.loginfo("Prequalification test failed: %s" % e)




if __name__ == "__main__":
    main()











