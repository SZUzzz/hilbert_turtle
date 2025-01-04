#include "ros/ros.h"
#include "turtlesim/Spawn.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include <cmath>

turtlesim::Pose group2_pose;
turtlesim::Pose group2_2_pose;
bool group2_received = false;
bool group2_2_received = false;

// 回调函数获取 Group2 的位置
void group2PoseCallback(const turtlesim::Pose::ConstPtr& msg)
{
    group2_pose = *msg;
    group2_received = true;
}

// 回调函数获取 Group2_2 的位置
void group2_2PoseCallback(const turtlesim::Pose::ConstPtr& msg)
{
    group2_2_pose = *msg;
    group2_2_received = true;
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "turtle_follower");
    ros::NodeHandle nh;

    // 1. 生成第二只小海龟 Group2_2
    ros::ServiceClient spawn_client = nh.serviceClient<turtlesim::Spawn>("spawn");
    turtlesim::Spawn spawn_srv;
    spawn_srv.request.x = 1.0;
    spawn_srv.request.y = 1.0;
    spawn_srv.request.name = "Group2_2";
    
    if (spawn_client.call(spawn_srv)) {
        ROS_INFO("小海龟 Group2_2 已成功生成");
    } else {
        ROS_ERROR("无法生成小海龟 Group2_2");
        return 1;
    }

    // 2. 订阅两只小海龟的位置
    ros::Subscriber group2_sub = nh.subscribe("/Group2/pose", 10, group2PoseCallback);
    ros::Subscriber group2_2_sub = nh.subscribe("/Group2_2/pose", 10, group2_2PoseCallback);

    // 3. 发布 Group2_2 的速度
    ros::Publisher velocity_pub = nh.advertise<geometry_msgs::Twist>("/Group2_2/cmd_vel", 10);

    ros::Rate rate(10);
    while (ros::ok())
    {
        ros::spinOnce();

        // 等待接收到两个小海龟的位置
        if (!group2_received || !group2_2_received) {
            rate.sleep();
            continue;
        }

        // 计算两只小海龟的距离
        double dx = group2_pose.x - group2_2_pose.x;
        double dy = group2_pose.y - group2_2_pose.y;
        double distance = std::sqrt(dx * dx + dy * dy);

        // 计算 Group2_2 朝向 Group2 的角度
        double target_angle = std::atan2(dy, dx);

        // 规范化角度到 [0, 2*PI]
        if (target_angle < 0) {
            target_angle += 2 * M_PI;
        }
        
        // 计算当前角度与目标角度的差值，并规范化到 [-PI, PI]
        double angle_diff = target_angle - group2_2_pose.theta;
        if (angle_diff > M_PI) {
            angle_diff -= 2 * M_PI;
        } else if (angle_diff < -M_PI) {
            angle_diff += 2 * M_PI;
        }

        // 计算速度，距离越大速度越快
        geometry_msgs::Twist vel_msg;
        if (distance < 0.05) {
            vel_msg.linear.x = 0.0;
            vel_msg.angular.z = 0.0;
        } else {
            vel_msg.linear.x = 0.5 * distance;      // 线速度与距离成正比
            vel_msg.angular.z = 4.0 * angle_diff;   // 角速度与角度差成正比
        }

        // 发布速度消息
        velocity_pub.publish(vel_msg);

        // 输出距离和角度
        ROS_INFO("两者之间的距离: %.2f, Group2_2 的朝向角度: %.2f", distance, group2_2_pose.theta);

        rate.sleep();
    }

    return 0;
}
