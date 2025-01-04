#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty
import random

def change_color():
    # 初始化 ROS 节点
    rospy.init_node('change_color', anonymous=True)

    # 创建服务客户端，用于调用 /clear 来刷新背景颜色
    clear_client = rospy.ServiceProxy('/clear', Empty)

    rate = rospy.Rate(1)  # 每秒一次

    while not rospy.is_shutdown():
        # 生成随机背景颜色
        r = random.randint(0, 255)
        g = random.randint(0, 255)
        b = random.randint(0, 255)

        # 设置参数以更改背景颜色
        rospy.set_param('/turtlesim/background_r', r)
        rospy.set_param('/turtlesim/background_g', g)
        rospy.set_param('/turtlesim/background_b', b)

        # 调用清除服务以应用新颜色
        try:
            clear_client()
            rospy.loginfo("背景颜色已更改为: R=%d, G=%d, B=%d", r, g, b)
        except rospy.ServiceException as e:
            rospy.logerr("无法调用 clear 服务: %s" % e)

        # 等待1秒
        rate.sleep()

if __name__ == '__main__':
    try:
        change_color()
    except rospy.ROSInterruptException:
        pass
