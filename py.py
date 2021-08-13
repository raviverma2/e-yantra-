#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import time
import math


theta1=0
radius=0
distance=0

def pose_callback(pose):

        global theta1
      
        theta1 = pose.theta
        
def move_turtle(lin_vel,ang_vel):

    global theta1,radius,distance

    rospy.init_node('turtle_revolve', anonymous=True)

    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

    
    rospy.Subscriber('/turtle1/pose',Pose, pose_callback)
    
    vel = Twist()
    
    
     
    radius=lin_vel/ang_vel
    
    distance = 2*math.pi*radius
    
    while not rospy.is_shutdown():
    
        vel.linear.x = lin_vel
        vel.linear.y = 0
        vel.linear.z = 0
        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = ang_vel
        
       
        if(theta1<0):
                
                 
                rospy.loginfo("Moving in circle")
                
                theta1 = theta1 + 2*(math.pi)
                
               
 
                if(distance-theta1*radius<0.02):
                
                    vel.linear.x=0.0
                    
                    vel.angular.z=0.0
                
                    pub.publish(vel)
                
                    rospy.logwarn("Stopping robot")
                
                    break
                
        pub.publish(vel)
        
        rospy.loginfo("Moving in circle")
        
if __name__ == '__main__':
    try:
        move_turtle(1.0,1.0)
    except rospy.ROSInterruptException:
        pass
