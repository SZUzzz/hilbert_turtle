#!/usr/bin/env python

import rospy
from turtlesim.srv import Spawn
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math

# 定义全局变量以存储两只小海龟的位置
group2_pose = None
group2_2_pose = None

# 回调函数获取 Group2 的位置
def group2_pose_callback(msg):
    global group2_pose
    group2_pose = msg

# 回调函数获取 Group2_2 的位置
def group2_2_pose_callback(msg):
    global group2_2_pose
    group2_2_pose = msg

def main():
    rospy.init_node('turtle_follower')

    # 1. 生成第二只小海龟 Group2_2
    rospy.wait_for_service('/spawn')
    try:
        spawn_client = rospy.ServiceProxy('/spawn', Spawn)
        spawn_client(1.0, 1.0, 0, 'Group2_2')
        rospy.loginfo("小海龟 Group2_2 已成功生成")
    except rospy.ServiceException as e:
        rospy.logerr("无法生成小海龟 Group2_2: %s" % e)
        return

    # 2. 订阅两只小海龟的位置
    rospy.Subscriber('/Group2/pose', Pose, group2_pose_callback)
    rospy.Subscriber('/Group2_2/pose', Pose, group2_2_pose_callback)

    # 3. 发布 Group2_2 的速度
    velocity_pub = rospy.Publisher('/Group2_2/cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # 确保两个小海龟的位置已被接收
        if group2_pose is None or group2_2_pose is None:
            rate.sleep()
            continue

        # 计算两只小海龟的距离
        dx = group2_pose.x - group2_2_pose.x
        dy = group2_pose.y - group2_2_pose.y
        distance = math.sqrt(dx**2 + dy**2)

        # 计算 Group2_2 朝向 Group2 的角度
        target_angle = math.atan2(dy, dx)

        # 规范化角度到 [0, 2*PI]
        if target_angle < 0:
            target_angle += 2 * math.pi

        # 计算当前角度与目标角度的差值，并规范化到 [-PI, PI]
        angle_diff = target_angle - group2_2_pose.theta
        if angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        elif angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        # 根据距离和角度差计算速度
        vel_msg = Twist()
        if distance < 0.05:
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0
        else:
            vel_msg.linear.x = 0.5 * distance  # 线速度与距离成正比
            vel_msg.angular.z = 4.0 * angle_diff  # 角速度与角度差成正比

        # 发布速度消息
        velocity_pub.publish(vel_msg)

        # 输出距离和角度
        rospy.loginfo("两者之间的距离: %.2f, Group2_2 的朝向角度: %.2f", distance, group2_2_pose.theta)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
