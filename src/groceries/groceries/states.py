
#!/usr/bin/env python
import math
import actionlib
import rospy
import smach
import smach_ros
import tf

from geometry_msgs.msg import Point, PoseStamped, Quaternion 
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class MoveToTable(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
        #self.omni_base = robot.try_get('omni_base')
    def execute(self, userdata):

        goal_x = 0.0 
        goal_y = 0.0 
        goal_yaw = 0.0 

        navigation_action(goal_x, goal_y, goal_yaw)


        return 'succeeded'

class MoveToShelf(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
        #self.omni_base = robot.try_get('omni_base')
    def execute(self, userdata):

        goal_x = 0.0 
        goal_y =  -0.6 
        goal_yaw = 0.0 

        navigation_action(goal_x, goal_y, goal_yaw)

        return 'succeeded'

class SelectObject(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
    def execute(self, userdata):
        
        return 'succeeded'
        
def navigation_action(goal_x,goal_y,goal_yaw):
	pose = PoseStamped()
	pose.header.stamp = rospy.Time.now()
	pose.header.frame_id = "map"
	pose.pose.position = Point(goal_x, goal_y, 0)
	quat = tf.transformations.quaternion_from_euler(0, 0, goal_yaw)
	pose.pose.orientation = Quaternion(*quat)

	goal = MoveBaseGoal()
	goal.target_pose = pose

	# send message to the action server
	cli.send_goal(goal)

	# wait for the action server to complete the order
	cli.wait_for_result()

	# print result of navigation
	result_action_state = cli.get_state()

	return result_action_state 


cli = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)
