#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point

from tf.transformations import euler_from_quaternion

import math

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from std_srvs.srv import *

posex=0.0
posey=0.0
pose=0.0
ebot_theta=0.0
e_theta=0.0
count_loop_=0
regions = None
x2=0.0
y2=0.0
state_desc_ = ['Go to point', 'wall following']
desired_position_=Point()
desired_position_.x = 12.5
desired_position_.y = 0
count_state_time_=0

srv_client_go_to_point_ = None
srv_client_wall_follower_ = None
yaw_ = 0
yaw_error_allowed_ = 5 * (math.pi / 180)

active_ = False
pub_ = None
regions_ = {
        'right': 0,
        'fright': 0,
        'front': 0,
        'fleft': 0,
        'left': 0,
}
state_ = 0
state_dict_ = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall',
}

def wall_follower_switch(req):
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res

def go_to_point_switch(req):
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res




def odom_callback(data):
    global posex, posey, pose, position_, yaw_

    position_ = data.pose.pose.position

    x  = data.pose.pose.orientation.x
    y  = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    posex = data.pose.pose.position.x
    posey=data.pose.pose.position.y
    pose= euler_from_quaternion([x,y,z,w])[2]

    yaw_ = euler_from_quaternion([x,y,z,w])[2]


def laser_callback(msg):
    global regions
    regions = {
        'right': min(min(msg.ranges[0:143]),10),
        'fright': min(min(msg.ranges[144:287]),10),
        'front':  min(min(msg.ranges[287:431]),10),
        'fleft':  min(min(msg.ranges[431:575]),10),
        'left':  min(min(msg.ranges[575:719]),10),
    }
    

def take_action():
    global regions_
    regions = regions_
    msg = Twist()
    linear_x = 0
    angular_z = 0
    
    state_description = ''
    
    d = 0.5
    
    if regions['front'] > d and regions['fleft'] > d and regions['fright'] > d:
        state_description = 'case 1 - nothing'
        change_state(0)
    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] > d:
        state_description = 'case 2 - front'
        change_state(1)
    elif regions['front'] > d and regions['fleft'] > d and regions['fright'] < d:
        state_description = 'case 3 - fright'
        change_state(2)
    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] > d:
        state_description = 'case 4 - fleft'
        change_state(0)
    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] < d:
        state_description = 'case 5 - front and fright'
        change_state(1)
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] > d:
        state_description = 'case 6 - front and fleft'
        change_state(1)
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] < d:
        state_description = 'case 7 - front and fleft and fright'
        change_state(1)
    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] < d:
        state_description = 'case 8 - fleft and fright'
        change_state(0)
    else:
        state_description = 'unknown case'
        
def find_wall():
    msg = Twist()
    msg.linear.x = 0.2
    msg.angular.z = -0.3
    return msg

def turn_left():
    msg = Twist()
    msg.angular.z = 0.3
    return msg

def follow_the_wall():
    global regions_
    
    msg = Twist()
    msg.linear.x = 0.5
    return msg

   

def change_state(state):
    global state_, state_desc_
    global srv_client_wall_follower_, srv_client_go_to_point_
    state_ = state
    
    if state_ == 0:
        resp = srv_client_go_to_point_(True)
        resp = srv_client_wall_follower_(False)
    if state_ == 1:
        resp = srv_client_go_to_point_(False)
        resp = srv_client_wall_follower_(True)




def fix_yaw(des_pos):
    global yaw_, pub, yaw_precision_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = normalize_angle(desired_yaw - yaw_)
    
    #rospy.loginfo(err_yaw)
    
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_:
        twist_msg.angular.z = 0.7 if err_yaw > 0 else -0.7
    
    pub.publish(twist_msg)
    
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_:
        #print 'Yaw error: [%s]' % err_yaw
        change_state(1)


def go_straight_ahead(des_pos):
    global yaw_, pub, yaw_precision_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) + pow(des_pos.x - position_.x, 2))
    
    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = 0.6
        twist_msg.angular.z = 0.2 if err_yaw > 0 else -0.2
        pub.publish(twist_msg)
    else:
        print 'Position error: [%s]' % err_pos
        change_state(2)
    
    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        print 'Yaw error: [%s]' % err_yaw
        change_state(0)


def done():
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub.publish(twist_msg)

        

def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle


def distance_to_line(p0):
    
    global initial_position_, desired_position_
    p1 = initial_position_
    p2 = desired_position_
    
    up_eq = math.fabs((p2.y - p1.y) * p0.x - (p2.x - p1.x) * p0.y + (p2.x * p1.y) - (p2.y * p1.x))
    lo_eq = math.sqrt(pow(p2.y - p1.y, 2) + pow(p2.x - p1.x, 2))
    distance = up_eq / lo_eq

    return distance
    
def control_loop():
    
    global regions, initial_position_, desired_position_, state_, yaw_, yaw_error_allowed_
    global srv_client_go_to_point_, srv_client_wall_follower_

    rospy.init_node('ebot_controller')
    
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    
    rate = rospy.Rate(10) 

    velocity_msg = Twist()
    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0
    pub.publish(velocity_msg)
    x1=0.0
    y1=0.0
    

    rospy.wait_for_service('/go_to_point_switch')
    rospy.wait_for_service('/wall_follower_switch')
    rospy.wait_for_service('/gazebo/set_model_state')
            
    srv_client_go_to_point_ = rospy.ServiceProxy('/go_to_point_switch', SetBool)
    srv_client_wall_follower_ = rospy.ServiceProxy('/wall_follower_switch', SetBool)
    srv_client_set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    
    change_state(0)      

    while not rospy.is_shutdown():
        
        if (x1<6.3): 
            x2=x1+((2*math.pi)/9)
            y2=2*(math.sin(x2))*(math.sin(x2/2))
            ebot_theta = math.atan2((y2-y1),(x2-x1))
            
            while (posex<=x2):

                e_theta=ebot_theta-pose
            
                v=0.2

                w=2.5*e_theta
                velocity_msg.linear.x = v
                velocity_msg.angular.z = w
                
                pub.publish(velocity_msg)
                rate.sleep()
                
                
                print 'ebot_theta', ebot_theta 
                
            x1=x2
            y1=y2
            xi=x1
            yi=y1

        if (x1>=6.3):
            
            

            
            initial_position_= Point()
            initial_position_.x = xi
            initial_position_.y = yi
            
   
            model_state = ModelState()
            model_state.model_name = 'ebot'
            model_state.pose.position.x = initial_position_.x
            model_state.pose.position.y = initial_position_.y
            resp = srv_client_set_model_state(model_state)


            distance_position_to_line = distance_to_line(position_)
            take_action()
            if regions == None:
                continue

            distance_position_to_line = distance_to_line(position_)

            if state_ == 0:
                if regions_['front'] > 0.15 and regions_['front'] < 1:
                    fix_yaw(desired_position_)
                    msg=find_wall()
                    change_state(1)

            elif state_ == 1:
                if count_state_time_ > 5 and \
                   distance_position_to_line < 0.1:
                   go_straight_ahead(desired_position_)
                   change_state(0)
            else: 
                
                done()
                
                
            count_loop_ = count_loop_ + 1
            if count_loop_ == 20:
                count_state_time_ = count_state_time_ + 1
                count_loop_ = 0
        
        
            rate.sleep()


if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass
