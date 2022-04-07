"""
.. module:: go_to_point
    :platform: Unix
    :synopsis: Node for making the robot move toward the goal
    
.. moduleauthor:: Maria Luisa Aiachini
  
This node implements a state machine to make the robot reach the goal position.

Publisher:
    /cmd_vel

Subscriber:
    /odom

Action Client:
    /go_to_point
"""

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

def clbk_odom(msg):
    """
    
    Callback for odometry subscriber it takes current position and
    orientation of the robot

    Args:
        msg: Message containind the data about the odometry of the robot
    
    """
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


def change_state(state):
    """
    Changing the state by assigning it to a global variable 
    
    Args:
        state: state that needs to be assigned to the global variable
    """
    
    global state_
    state_ = state
    print ('State changed to [%s]' % state_)

def normalize_angle(angle):
    """

    Normalizing the angle 

    Args:
        angle: Angle to be normalized

    """
    
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

def fix_yaw(des_pos):
    """

    Fixing the yaw to make the robot oriented toward the goal

    Args:
        des_pos: Goal position

    """
    
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

def go_straight_ahead(des_pos):
    """

    Making the robot going straight to reach the robot

    Args:
        des_pos: Goal position

    """
    
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) + pow(des_pos.x - position_.x, 2))
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

def fix_final_yaw(des_yaw):
    """

    Fixing the yaw at the robot once the goal is reached to match the needed orientation

    Args:
        des_yaw: Goal orientation of the robot

    """
    
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
      
def done():
    """

    Stopping the robot once the target position and orientation are reached

    """
    
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub_.publish(twist_msg)
    success = True
    act_s.set_succeeded()

def go_to_point(goal):
    """

    Callback for the action server

    Args:
        goal: Goal position

    Returns:
        Always true

    """

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

def main():
    """

    Main function of the node. In this function the node is initiated and are 
    defined all the publishers, subscribers and action clients.

    """
    
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
