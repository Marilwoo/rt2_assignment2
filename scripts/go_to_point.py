##
# \file go_to_point.py
# \brief Node for making the robot move toward the goal
# \author Maria Luisa Aiachini
# 
# \details
#
# \Publishes to: <BR>
#	°/cmd_vel
#
# \Subscribes to:<BR>
#	°/odom
#
# \Action Client: <BR>
#	°/go_to_point
#
# Description:
# This node implements a state machine to make the robot reach the goal position.
#

#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
import math
import actionlib
import actionlib.msg
import rt2_assignment1.msg

# robot state variables
desired_position_ = Point()
yaw_ = 0
position_ = 0
state_ = 0
pub_ = None
# action server
act_s = None

# parameters for control
yaw_precision_ = math.pi / 9  # +/- 20 degree allowed
yaw_precision_2_ = math.pi / 90  # +/- 2 degree allowed
dist_precision_ = 0.1
kp_a = -3.0 
kp_d = 0.2
ub_a = 0.6
lb_a = -0.5
ub_d = 0.6

##
# \brief Callback for the /odom subscriber
#
# \param msg: Message containind the data about the odometry of the robot
#
# Callback for odometry subscriber it takes current position and
# orientation of the robot
#

def clbk_odom(msg):
    global position_
    global yaw_

    # position
    position_ = msg.pose.pose.position

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]

##
# \brief Changing the state by assigning to the global variable state_
#
# \param state: State that needs to be assigned to the variable 'state_'
#

def change_state(state):
    global state_
    state_ = state
    print ('State changed to [%s]' % state_)

##
# \brief Normalizing the angle 
#
# \param angle: Angle to be normalized
#

def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

##
# \brief Fixing the yaw to make the robot oriented toward the goal
# 
# \param des_pos: Goal position
#

def fix_yaw(des_pos):
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = normalize_angle(desired_yaw - yaw_)
    rospy.loginfo(err_yaw)
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a*err_yaw
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = ub_a
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = lb_a
    pub_.publish(twist_msg)
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        change_state(1)

##
# \brief Making the robot going straight to reach the robot
#
# \param des_pos: Goal position
#

def go_straight_ahead(des_pos):
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) +
                        pow(des_pos.x - position_.x, 2))
    err_yaw = normalize_angle(desired_yaw - yaw_)
    rospy.loginfo(err_yaw)

    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = 0.3
        if twist_msg.linear.x > ub_d:
            twist_msg.linear.x = ub_d

        twist_msg.angular.z = kp_a*err_yaw
        pub_.publish(twist_msg)
    else: # state change conditions
        change_state(2)

    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        change_state(0)
##
# \brief Fixing the yaw at the robot once the goal is reached to match the needed orientation
#
# \param des_yaw: Goal orientation of the robot
#

def fix_final_yaw(des_yaw):
    err_yaw = normalize_angle(des_yaw - yaw_)
    rospy.loginfo(err_yaw)
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a*err_yaw
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = ub_a
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = lb_a
    pub_.publish(twist_msg)
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        change_state(3)
##
# \brief Stopping the robot once the target position and orientation are reached
#
      
def done():
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub_.publish(twist_msg)
    success = True
    act_s.set_succeeded()

##
# \brief Callback for the action server
#
# \param goal: Goal position
#
# \return Always true
#

def go_to_point(goal):
	global state_, desired_position_, act_s, success
	print("received")
    	
	# Getting the desired position and orientation
	desired_position_.x = goal.target_pose.pose.position.x
	print ("desired position x: %s" %desired_position_.x)
	desired_position_.y = goal.target_pose.pose.position.y
	des_yaw = goal.target_pose.pose.orientation.z
    	
	# Changing the state
	change_state(0)
	success = True
    
	while True:
		# Checking if the request is preempted. If it is assigning the velocities to 0
		# to stop immediately the robot
		if act_s.is_preempt_requested():
			rospy.loginfo('Goal was preempted')
			twist_msg = Twist()
			twist_msg.linear.x = 0
			twist_msg.angular.z = 0
			pub_.publish(twist_msg)
			act_s.set_preempted()
			success = False
			break
		# Checking the state and calling the corresponding function
		elif state_ == 0:
			fix_yaw(desired_position_)
		elif state_ == 1:
			go_straight_ahead(desired_position_)
		elif state_ == 2:
			fix_final_yaw(des_yaw)
		elif state_ == 3:
			done()
			break
	return True

##
# \brief Main function of the node
#
# In this function the node is initiated and are defined all the publishers, subscribers
# and action clients.
#

def main():
	global pub_, act_s
	rospy.init_node('go_to_point')
	pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
	sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    
	# Defining the ation server for the /go_to_point
	act_s = actionlib.SimpleActionServer('/go_to_point', rt2_assignment1.msg.PlanningAction, go_to_point, auto_start=False)
	act_s.start()
	rospy.spin()

if __name__ == '__main__':
    main()
