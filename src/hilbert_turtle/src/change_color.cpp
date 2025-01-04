#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include <cstdlib>
#include <ctime>

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    // 初始化 ROS 节点
    ros::init(argc, argv, "change_color");
    ros::NodeHandle nh;

    // 服务客户端，用于调用 /clear 来刷新背景颜色
    ros::ServiceClient clear_client = nh.serviceClient<std_srvs::Empty>("/clear");
    std_srvs::Empty clear_srv;

    // 设置随机数种子
    std::srand(std::time(nullptr));

    ros::Rate rate(1);

    while (ros::ok())
    {
        // 生成随机背景颜色
        int r = std::rand() % 256;
        int g = std::rand() % 256;
        int b = std::rand() % 256;

        // 设置参数以更改背景颜色
        nh.setParam("/turtlesim/background_r", r);
        nh.setParam("/turtlesim/background_g", g);
        nh.setParam("/turtlesim/background_b", b);

        // 调用清除服务以应用新颜色
        if (clear_client.call(clear_srv))
        {
            ROS_INFO("背景颜色已更改为: R=%d, G=%d, B=%d", r, g, b);
        }
        else
        {
            ROS_ERROR("无法调用 clear 服务");
        }

        // 等待1秒
        rate.sleep();
    }

    return 0;
}
