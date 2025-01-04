#include "ros/ros.h"
#include "geometry_msgs/Twist.h"    
#include "turtlesim/Pose.h"
#include <cmath>

/**
 * @brief move
 * 
 * @param pub 
 * @param speed 
 * @param distance 
 * @param isForward 
 */
void moveTurtle(ros::Publisher &pub, double speed, double distance, bool isForward)
{
    geometry_msgs::Twist vel_msg;
    if(isForward) {
        vel_msg.linear.x = fabs(speed);
    } else {
        vel_msg.linear.x = -fabs(speed);
    }
    double t0 = ros::Time::now().toSec();
    double current_distance = 0.0;

    ros::Rate loop_rate(100);
    while(current_distance < distance)
    {
        pub.publish(vel_msg);
        double t1 = ros::Time::now().toSec();
        current_distance = speed * (t1-t0);
        loop_rate.sleep();
    }
    vel_msg.linear.x = 0.0;
    pub.publish(vel_msg);
}

/**
 * @brief rotate
 * 
 * @param pub 
 * @param angular_speed 
 * @param angle 
 * @param clockwise 
 */
void rotateTurtle(ros::Publisher &pub, double angular_speed, double angle, bool clockwise)
{
    geometry_msgs::Twist vel_msg;
    if(clockwise) {
        vel_msg.angular.z = -fabs(angular_speed);  // 顺时针为负
    } else {
        vel_msg.angular.z = fabs(angular_speed);   // 逆时针为正
    }
    double t0 = ros::Time::now().toSec();
    double current_angle = 0.0;

    ros::Rate loop_rate(100);
    while(current_angle < angle)
    {
        pub.publish(vel_msg);
        double t1 = ros::Time::now().toSec();
        current_angle = angular_speed * (t1-t0);
        loop_rate.sleep();
    }
    vel_msg.angular.z = 0.0;
    pub.publish(vel_msg);
}

/**
 * @brief Hilbert Curve Recursion
 * 
 * @param pub 
 * @param order 阶数
 * @param size  每段长度
 * @param angle_rotation Hilbert曲线的转向角度
 * @param direction 是否逆时针方向
 */
void hilbert(ros::Publisher &pub, int order, double size, double angle_rotation, bool direction)
{
    if(order == 0) return;

    // Rotate and move to follow the pattern of Hilbert curves
    if(direction) {
        rotateTurtle(pub, M_PI/4, M_PI/2, true);
        hilbert(pub, order - 1, size, angle_rotation, !direction);
        moveTurtle(pub, 1.0, size, true);
        
        rotateTurtle(pub, M_PI/4, M_PI/2, false);
        hilbert(pub, order - 1, size, angle_rotation, direction);
        moveTurtle(pub, 1.0, size, true);
        
        hilbert(pub, order - 1, size, angle_rotation, direction);
        rotateTurtle(pub, M_PI/4, M_PI/2, false);
        moveTurtle(pub, 1.0, size, true);
        
        hilbert(pub, order - 1, size, angle_rotation, !direction);
        rotateTurtle(pub, M_PI/4, M_PI/2, true);
    } else {
        rotateTurtle(pub, M_PI/4, M_PI/2, false);
        hilbert(pub, order - 1, size, angle_rotation, !direction);
        moveTurtle(pub, 1.0, size, true);
        
        rotateTurtle(pub, M_PI/4, M_PI/2, true);
        hilbert(pub, order - 1, size, angle_rotation, direction);
        moveTurtle(pub, 1.0, size, true);
        
        hilbert(pub, order - 1, size, angle_rotation, direction);
        rotateTurtle(pub, M_PI/4, M_PI/2, true);
        moveTurtle(pub, 1.0, size, true);
        
        hilbert(pub, order - 1, size, angle_rotation, !direction);
        rotateTurtle(pub, M_PI/4, M_PI/2, false);
    }
}

/**
 * @brief Go by Hilbert curve
 * 
 * @param pub 
 * @param order 阶数
 * @param size  每段长度
 */
void hilbertCurve(ros::Publisher &pub, int order, double size)
{
    hilbert(pub, order, size, M_PI/2, true);
}

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"hilbert_turtle");
    ros::NodeHandle nh;

    // Publisher -> control turtle
    ros::Publisher velocity_pub = nh.advertise<geometry_msgs::Twist>("/Group2/cmd_vel",10);

    ros::Rate loop_rate(100);

    ros::Duration(2.0).sleep();
    
    int order = 2;
    double size = 0.8;

    // Start drawing the Hilbert curve
    hilbertCurve(velocity_pub, order, size);
    
    ros::spin();
    return 0;
}
