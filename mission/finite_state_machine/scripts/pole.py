import rospy
import smach
from smach import StateMachine
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from std_msgs.msg import String
from landmarks.srv import request_position

import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal, MoveBaseGoal
from vortex_msgs.msg import VtfPathFollowingAction, VtfPathFollowingGoal, SetVelocityGoal, SetVelocityAction, DpSetpoint
from landmarks.srv import request_position
from tf.transformations import quaternion_from_euler
from fsm_helper import dp_move, get_pose_in_front, los_move, create_circle_coordinates
from vortex_msgs.srv import ControlMode
from nav_msgs.msg import Odometry

class PoleSearch(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['preempted', 'succeeded', 'aborted'])           
        self.landmarks_client = rospy.ServiceProxy('send_positions',request_position)
        self.pole_position = Pose()
        self.is_detected = False
        self.state_pub = rospy.Publisher('/fsm/state',String,queue_size=1)

        vtf_action_server = "/controllers/vtf_action_server"
        self.vtf_client = actionlib.SimpleActionClient(vtf_action_server, VtfPathFollowingAction)  

        
    def execute(self, userdata):
        self.state_pub.publish("pole_search")

        goal = VtfPathFollowingGoal()
        goal.waypoints = [Point(15,0,-0.5)]
        goal.forward_speed = 0.2
        goal.heading = "path_dependent_heading"
        self.vtf_client.wait_for_server()
        self.vtf_client.send_goal(goal)
        rate = rospy.Rate(10)

        while not self.is_detected:
            print("SEARCHING FOR POLE ...")
            rospy.wait_for_service('send_positions')
            self.pole_position = self.landmarks_client("pole").object.objectPose.pose.position
            self.is_detected = self.landmarks_client("pole").object.isDetected
            rate.sleep()

        self.vtf_client.cancel_all_goals()
        print("POLE POSITION DETECTED: "+ str(self.pole_position.x) + ", "+ str(self.pole_position.y)+ ", "+ str(self.pole_position.z))    
                           
        return 'succeeded'
    
    

class PoleConverge(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['preempted', 'succeeded', 'aborted'], output_keys=['pole_converge_output'])
        
        self.landmarks_client = rospy.ServiceProxy('send_positions',request_position)  
        rospy.wait_for_service('send_positions')
        self.object = self.landmarks_client("pole").object

        dp_guidance_action_server="/guidance_interface/dp_server" 
        self.action_client = actionlib.SimpleActionClient(dp_guidance_action_server, MoveBaseAction)   

        self.dp_pub = rospy.Publisher("/controllers/dp_data", DpSetpoint, queue_size=1)
        self.state_pub = rospy.Publisher('/fsm/state',String,queue_size=1)

        vtf_action_server = "/controllers/vtf_action_server"
        self.vtf_client = actionlib.SimpleActionClient(vtf_action_server, VtfPathFollowingAction) 

        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_cb)
        self.odom = Odometry()

    def odom_cb(self, msg):
        self.odom = msg   
   

    def execute(self, userdata):
        self.state_pub.publish("pole_converge")

        goal = VtfPathFollowingGoal()
        self.object = self.landmarks_client("pole").object
        goal_pose = get_pose_in_front(self.object.objectPose.pose, 0.5)

        print('get_pose_in_front returned:')
        print(goal_pose)

        goal.waypoints = [goal_pose.position]
        goal.forward_speed = 0.1
        goal.heading = "path_dependent_heading"

        self.vtf_client.wait_for_server()
        self.action_client.cancel_all_goals()
        self.vtf_client.send_goal(goal)
        rate = rospy.Rate(1)
        rate.sleep()
        while not rospy.is_shutdown():
            if self.vtf_client.simple_state == actionlib.simple_action_client.SimpleGoalState.DONE:
                break
            self.object = self.landmarks_client("pole").object
            
            goal.waypoints = [self.object.objectPose.pose.position]
            print("POLE POSITION DETECTED: "+ str(goal.waypoints[0].x) + ", "+ str(goal.waypoints[0].y)+ ", "+ str(goal.waypoints[0].z))

            goal.waypoints[0] = get_pose_in_front(self.object.objectPose.pose, 0.5).position

            self.vtf_client.send_goal(goal)
            userdata.pole_converge_output = self.object
            rate.sleep()

            if self.object.estimateFucked:
                self.vtf_client.cancel_all_goals()
                return 'aborted'
        self.vtf_client.cancel_all_goals()

        dp_goal = DpSetpoint()
        dp_goal.control_mode = 7 # POSE_HOLD
        dp_goal.setpoint = self.odom.pose.pose
        self.dp_pub.publish(dp_goal)
        while not rospy.is_shutdown() and not self.object.estimateConverged:
            self.object = self.landmarks_client("pole").object
            if self.object.estimateFucked:
                dp_goal.control_mode = 0 # OPEN_LOOP
                self.dp_pub.publish(dp_goal)
                return 'aborted'
            rate.sleep()
        dp_goal.control_mode = 0 # OPEN_LOOP
        self.dp_pub.publish(dp_goal)
        self.object = self.landmarks_client("pole").object
        userdata.pole_converge_output=self.object
        print("POLE POSITION ESTIMATE CONVERGED AT: " + str(self.object.objectPose.pose.position.x) + "; " \
        + str(self.object.objectPose.pose.position.y) + "; " \
        + str(self.object.objectPose.pose.position.z))    

        return 'succeeded'

class PoleExecute(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['preempted', 'succeeded', 'aborted'],input_keys=['pole'])  
        
        vtf_action_server = "/controllers/vtf_action_server"
        self.vtf_client = actionlib.SimpleActionClient(vtf_action_server, VtfPathFollowingAction) 

        self.state_pub = rospy.Publisher('/fsm/state',String,queue_size=1)

        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_cb)
        self.odom = Odometry()

    def odom_cb(self, msg):
        self.odom = msg    

    def execute(self, userdata):
        self.state_pub.publish("pole_execute")

        goal = VtfPathFollowingGoal()
        start = self.odom.pose.pose.position
        print(userdata)
        centre = Point(userdata.pole.objectPose.pose.position.x,userdata.pole.objectPose.pose.position.y, userdata.pole.objectPose.pose.position.z)
        goal.waypoints = create_circle_coordinates(start,centre,330)
        goal.forward_speed = 0.1
        goal.heading = "path_dependent_heading"

        self.vtf_client.wait_for_server()
        self.vtf_client.send_goal(goal)
        rate = rospy.Rate(1)
        rate.sleep()
        while not rospy.is_shutdown():
            if self.vtf_client.simple_state == actionlib.simple_action_client.SimpleGoalState.DONE:
                break
            rate.sleep()
        return 'succeeded'