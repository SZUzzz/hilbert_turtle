#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import math
import time

def move_turtle(pub, speed, distance, is_forward):
    vel_msg = Twist()
    vel_msg.linear.x = abs(speed) if is_forward else -abs(speed)
    
    t0 = rospy.Time.now().to_sec()
    current_distance = 0.0
    
    rate = rospy.Rate(100)
    while current_distance < distance and not rospy.is_shutdown():
        pub.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_distance = speed * (t1 - t0)
        rate.sleep()
    
    vel_msg.linear.x = 0.0
    pub.publish(vel_msg)

def rotate_turtle(pub, angular_speed, angle, clockwise):
    vel_msg = Twist()
    vel_msg.angular.z = -abs(angular_speed) if clockwise else abs(angular_speed)
    
    t0 = rospy.Time.now().to_sec()
    current_angle = 0.0
    
    rate = rospy.Rate(100)
    while current_angle < angle and not rospy.is_shutdown():
        pub.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_angle = angular_speed * (t1 - t0)
        rate.sleep()
    
    vel_msg.angular.z = 0.0
    pub.publish(vel_msg)

def hilbert(pub, order, size, angle_rotation, direction):
    if order == 0:
        return
    
    if direction:
        rotate_turtle(pub, math.pi / 2, math.pi / 2, True)
        hilbert(pub, order - 1, size, angle_rotation, not direction)
        move_turtle(pub, 1.0, size, True)
        
        rotate_turtle(pub, math.pi / 2, math.pi / 2, False)
        hilbert(pub, order - 1, size, angle_rotation, direction)
        move_turtle(pub, 1.0, size, True)
        
        hilbert(pub, order - 1, size, angle_rotation, direction)
        rotate_turtle(pub, math.pi / 2, math.pi / 2, False)
        move_turtle(pub, 1.0, size, True)
        
        hilbert(pub, order - 1, size, angle_rotation, not direction)
        rotate_turtle(pub, math.pi / 2, math.pi / 2, True)
    else:
        rotate_turtle(pub, math.pi / 2, math.pi / 2, False)
        hilbert(pub, order - 1, size, angle_rotation, not direction)
        move_turtle(pub, 1.0, size, True)
        
        rotate_turtle(pub, math.pi / 2, math.pi / 2, True)
        hilbert(pub, order - 1, size, angle_rotation, direction)
        move_turtle(pub, 1.0, size, True)
        
        hilbert(pub, order - 1, size, angle_rotation, direction)
        rotate_turtle(pub, math.pi / 2, math.pi / 2, True)
        move_turtle(pub, 1.0, size, True)
        
        hilbert(pub, order - 1, size, angle_rotation, not direction)
        rotate_turtle(pub, math.pi / 2, math.pi / 2, False)

def hilbert_curve(pub, order, size):
    hilbert(pub, order, size, math.pi / 2, True)

if __name__ == '__main__':
    rospy.init_node('hilbert_turtle', anonymous=True)
    velocity_pub = rospy.Publisher('/Group2/cmd_vel', Twist, queue_size=10)
    
    rospy.sleep(2.0)
    
    order = 2
    size = 0.8
    
    hilbert_curve(velocity_pub, order, size)
    
    rospy.spin()
