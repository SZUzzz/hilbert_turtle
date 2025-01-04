#include "ros/ros.h"
#include "turtlesim/Pose.h"

void poseCallback(const turtlesim::Pose::ConstPtr &msg)
{
    ROS_INFO("乌龟当前位置:(%.2f,%.2f),朝向:%.2f", msg->x, msg->y, msg->theta);
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    // 2. 初始化ros节点
    ros::init(argc, argv, "print_pose");
    // 3. 创建ros句柄
    ros::NodeHandle nh;
    // 4. 创建订阅者，并使用标准回调函数
    ros::Subscriber sub = nh.subscribe("/Group2/pose", 100, poseCallback);
    // 5. 处理订阅
    ros::spin();
    return 0;
}
