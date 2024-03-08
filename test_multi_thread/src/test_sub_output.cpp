#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <ros/time.h>

// 回调函数，用于处理接收到的消息
void messageCallback(const std_msgs::Float64::ConstPtr& msg) {
    // 获取当前时间
    ros::Time receive_time = ros::Time::now();

    // 打印接收到的消息和接收时间，精确到小数点后3位
    ROS_INFO("Received message: %f at time %.3f s", msg->data, receive_time.toSec());
}

int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "subscriber_node");
    ros::NodeHandle nh;

    // 创建一个ROS订阅者，订阅名为"velocity"的话题，消息类型为std_msgs::Float64
    ros::Subscriber sub = nh.subscribe("velocity", 1, messageCallback);

    // 循环等待消息到来
    ros::spin();

    return 0;
}