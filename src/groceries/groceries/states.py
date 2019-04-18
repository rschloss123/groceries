
#!/usr/bin/env python
import math
import actionlib
import rospy
import smach
import smach_ros
import tf

from geometry_msgs.msg import Point, PoseStamped, Quaternion 
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from manip_prelim.msg import arPoseAction, arPoseGoal

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
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'], input_keys=[], output_keys = ['object_location'])
        ar_cli.wait_for_server()

    def execute(self, userdata):
        check_ar_marker_identified = '' 

        while(check_ar_marker_identified == ''):

            select_goal = arPoseGoal()
            select_goal.samples = 1
            ar_cli.send_goal(select_goal)

            ar_cli.wait_for_result()

            select_result = ar_cli.get_result()

            #print "select result"
            #print select_result 
            #exit()

            check_ar_marker_identified = select_result.arpose.header.frame_id
            if check_ar_marker_identified == '':
                print "no ar marker"
        print "select result" 
        print select_result


        target_pose = PoseStamped()

        target_pose.pose.position.x = select_result.arpose.pose.position.x
        target_pose.pose.position.y = select_result.arpose.pose.position.y
        target_pose.pose.position.z = select_result.arpose.pose.position.z
        target_pose.pose.orientation.x = select_result.arpose.pose.orientation.x
        target_pose.pose.orientation.y = select_result.arpose.pose.orientation.y
        target_pose.pose.orientation.z = select_result.arpose.pose.orientation.z
        target_pose.pose.orientation.w = select_result.arpose.pose.orientation.w
        target_pose.header.frame_id = select_result.arpose.header.frame_id

        userdata.object_location = target_pose

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
ar_cli = actionlib.SimpleActionClient('averaging', arPoseAction)
