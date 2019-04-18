import actionlib
import geometry_msgs
import rospy
import smach 
import tf
from hsr_manipulation_2019.msg import *

class PickObject(smach.State):
    def __init__(self, robot):

        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'], input_keys=["object_location"])
        self.pickUpClient = actionlib.SimpleActionClient('pickUpMoveitAction', hsr_manipulation_2019.msg.pickUpMoveitAction)
 #       self.listener = tf.TransformListener()
 #       self.whole_body = robot.try_get("whole_body")

    def execute(self, userdata):
        got_server = self.pickUpClient.wait_for_server(rospy.Duration(10))
        if not got_server:
            rospy.logerr("Could not connect to pick up server")
            return "aborted"

        goal = hsr_manipulation_2019.msg.pickUpMoveitGoal()

#        goal.target_pose.pose.position.x = 0.8
#        goal.target_pose.pose.position.y = 0.0 #0.4
#        goal.target_pose.pose.position.z = 1.0 #0.5
#        goal.target_pose.pose.orientation.x = 0.0
#        goal.target_pose.pose.orientation.y = 0.0
#        goal.target_pose.pose.orientation.z = 0.0
#        goal.target_pose.pose.orientation.w = 0.0
#        goal.target_pose.header.frame_id  = 'map' # 'head_rgbd_sensor_link'

        goal.target_pose.pose.position.x = userdata.object_location.pose.position.x 
        goal.target_pose.pose.position.y = userdata.object_location.pose.position.y 
        goal.target_pose.pose.position.z = userdata.object_location.pose.position.z 
        goal.target_pose.pose.orientation.x = userdata.object_location.pose.orientation.x 
        goal.target_pose.pose.orientation.y = userdata.object_location.pose.orientation.y 
        goal.target_pose.pose.orientation.z = userdata.object_location.pose.orientation.z 
        goal.target_pose.pose.orientation.w = userdata.object_location.pose.orientation.w 
        goal.target_pose.header.frame_id  = userdata.object_location.header.frame_id 

        print "requesting" 

        self.pickUpClient.send_goal(goal)
        self.pickUpClient.wait_for_result()

        return 'succeeded' # TODO return aborted:w


#        loc_st = userdata.object_location
#        self.listener.waitForTransform('/head_rgbd_sensor_link', loc_st.header.frame_id, rospy.Time(0),
#                                       rospy.Duration(4.0))
#        base_link_point = self.listener.transformPoint('head_rgbd_sensor_link', loc_st).point
#
#        goal.object_location = base_link_point
#
#        self.pickUpClient.send_goal(goal)
#        self.pickUpClient.wait_for_result(rospy.Duration.from_sec(45.0))
#        # TODO this doesnt work
#        state = self.pickUpClient.get_state()
#
#        # if state == GoalStatus.SUCCEEDED:
#        if self.whole_body.joint_positions['hand_motor_joint'] > -0.4:
#            return 'succeeded'
#        else:
#            return 'aborted'
#

class PlaceObject(smach.State):
    def __init__(self, robot):

        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'], input_keys=["object_location"])
        self.putDownClient = actionlib.SimpleActionClient('putDownMoveitAction', hsr_manipulation_2019.msg.putDownMoveitAction)

    def execute(self, userdata):
        got_server = self.putDownClient.wait_for_server(rospy.Duration(10))
        if not got_server:
            rospy.logerr("Could not connect to pick up server")
            return "aborted"

        goal = hsr_manipulation_2019.msg.putDownMoveitGoal()

        goal.target_pose.pose.position.x = 0.7
        goal.target_pose.pose.position.y = -0.6 #0.4
        goal.target_pose.pose.position.z = 1.15 #0.5
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 0.0
        goal.target_pose.header.frame_id  = 'map' #head_rgbd_sensor_link'

        print "requesting" 

        self.putDownClient.send_goal(goal)
        self.putDownClient.wait_for_result()

        return 'succeeded' # TODO return aborted:w


