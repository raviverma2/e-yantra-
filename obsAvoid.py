#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from tf.transformations import euler_from_quaternion

import math

posex=0.0
posey=0.0
pose=0.0
ebot_theta=0.0
e_theta=0.0
m_theta=0
desx1=12.5
desx2=0
x2=0.0
y2=0.0
m=0

def odom_callback(data):
    global posex, posey, pose
    x  = data.pose.pose.orientation.x
    y  = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    posex = data.pose.pose.position.x
    posey=data.pose.pose.position.y
    pose= euler_from_quaternion([x,y,z,w])[2]


def laser_callback(msg):
    global regions
    regions = {
        'right': min(min(msg.ranges[0:143]),10),
        'fright': min(min(msg.ranges[144:287]),10),
        'front':  min(min(msg.ranges[287:431]),10),
        'fleft':  min(min(msg.ranges[431:575]),10),
        'left':  min(min(msg.ranges[575:719]),10),
    }




def control_loop():
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
                print 'Tracing_curve'
                
                 
                
            x1=x2
            y1=y2
            
        if (x1>=6.3):


            
            print 'obstacle_case'
            state_description = ''
    
            if regions['front'] > 1.5 and regions['fleft'] > 1.5 and regions['fright'] > 1.5:
                state_description = 'case 1 - nothing'
                

                m_theta= -pose+ math.atan2(0-posey, 12.5-posex)
                
                m=0.9*m_theta
                 
                velocity_msg.linear.x = 0.4
                velocity_msg.angular.z = m

                
                
            elif regions['front'] < 1.5 and regions['fleft'] > 1.5 and regions['fright'] > 1.5:
                state_description = 'case 2 - front'
                velocity_msg.linear.x = 0
                velocity_msg.angular.z = 1

            elif regions['front'] > 1.5 and regions['fleft'] > 1.5 and regions['fright'] < 1.5:
                state_description = 'case 3 - fright'
                velocity_msg.linear.x = 0
                velocity_msg.angular.z = 0
            elif regions['front'] > 1.5 and regions['fleft'] < 1.5 and regions['fright'] > 1.5:
                state_description = 'case 4 - fleft'
                velocity_msg.linear.x = 0
                velocity_msg.angular.z = -0.3
            elif regions['front'] < 1.5 and regions['fleft'] > 1.5 and regions['fright'] < 1.5:
                state_description = 'case 5 - front and fright'
                velocity_msg.linear.x = 0
                velocity_msg.angular.z = 1
            elif regions['front'] < 1.5 and regions['fleft'] < 1.5 and regions['fright'] > 1.5:
                state_description = 'case 6 - front and fleft'
                velocity_msg.linear.x = 0
                velocity_msg.angular.z = -0.6
            elif regions['front'] < 1.5 and regions['fleft'] < 1.5 and regions['fright'] < 1.5:
                state_description = 'case 7 - front and fleft and fright'
                velocity_msg.linear.x = 0
                velocity_msg.angular.z = 1
            elif regions['front'] > 1.5 and regions['fleft'] < 1.5 and regions['fright'] < 1.5:
                state_description = 'case 8 - fleft and fright'
                velocity_msg.linear.x = 0.6
                velocity_msg.angular.z = 0
            else:
                state_description = 'unknown case'
                rospy.loginfo(regions)

            rospy.loginfo(state_description)
            pub.publish(velocity_msg)
            print 'State: ',state_description
            rate.sleep()
            

    	
    	rate.sleep()

    	if (posex>=12.5):
            print' goal_reached'
            velocity_msg.linear.x = 0.0
            velocity_msg.angular.z = 0
            
            pub.publish(velocity_msg)
            
            break

if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass
