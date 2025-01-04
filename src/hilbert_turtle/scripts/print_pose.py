#!/usr/bin/env python

import rospy
from turtlesim.msg import Pose

def pose_callback(msg):
    rospy.loginfo("乌龟当前位置:(%.2f, %.2f), 朝向: %.2f" % (msg.x, msg.y, msg.theta))

if __name__ == '__main__':
    # 初始化 ROS 节点
    rospy.init_node('print_pose', anonymous=True)
    
    # 创建订阅者，并使用标准回调函数
    rospy.Subscriber('/Group2/pose', Pose, pose_callback)
    
    # 处理订阅
    rospy.spin()
