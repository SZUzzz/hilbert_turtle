#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from turtlesim.srv import Spawn

def spawn_turtle():
    # 初始化 ROS 节点
    rospy.init_node("set_turtle", anonymous=True)
    # 等待服务启动
    rospy.wait_for_service("/spawn")
    try:
        # 创建服务客户端
        spawn_client = rospy.ServiceProxy("/spawn", Spawn)
        # 设置请求参数
        x = 10.0
        y = 8.0
        theta = -3.14
        name = "Group2"
        
        # 发送请求并处理响应
        response = spawn_client(x, y, theta, name)
        rospy.loginfo("新的乌龟生成, 名字: %s", response.name)
    except rospy.ServiceException as e:
        rospy.loginfo("乌龟生成失败: %s", e)

if __name__ == "__main__":
    spawn_turtle()
