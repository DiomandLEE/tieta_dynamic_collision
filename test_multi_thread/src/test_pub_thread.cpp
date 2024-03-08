#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>

std::vector<double> vel_; // 用于存储要发布的数据
std::mutex vel_mutex; // 互斥量，用于保护共享数据vel_
std::mutex stop_mutex; // 互斥量，用于保护停止标志stop_publishing
std::condition_variable cv; // 条件变量，用于线程间通信
bool should_publish = false; // 发布标志，用于指示是否需要发布新的数据
bool stop_publishing = false; // 终止发布标志，用于指示是否需要终止当前的发布
bool update_flag = false; // 更新标志，用于指示是否需要更新数据

// 发布消息的函数
void publishMessages(ros::Publisher& pub, ros::NodeHandle & nh) {
    std::cout << "Publishing messages..." << std::endl;
    ros::Rate rate(10); // 设置发布频率为10Hz

    int i = 0;

    // 等待条件变量通知，开始发布消息
    while (ros::ok()) {
        std_msgs::Float64 msg;

        // 等待直到有数据可发布或者收到终止发布的信号
        // {
        //     std::unique_lock<std::mutex> lock(vel_mutex); // 上锁
        //     cv.wait(lock, []{ return should_publish || stop_publishing; });
        //     if (stop_publishing) {
        //         stop_publishing = false; // 重置终止发布标志
        //         should_publish = false; // 重置发布标志
        //         continue; // 终止当前遍历，开始新的遍历
        //     }
        std::cout << "Publishing new message..." << update_flag << std::endl;
        // } // 解锁
            {

                //std::lock_guard<std::mutex> lock1_(stop_mutex); // 上锁

                if(update_flag)
                {
                    update_flag = false;
                    i = 0; // 重置索引
                    ROS_ERROR("Updating data...");
                    // 更新数据
                }

            }

        // 锁住互斥量，保证线程安全地访问vel_
        {
            //std::lock_guard<std::mutex> lock1(vel_mutex); // 上锁

            msg.data = vel_[i]; // 获取vel_中的数据

            std::cout << "Publishing!!!! "<< std::endl;

            pub.publish(msg); // 发布消息


            i++;

            rate.sleep();





        // // 从新的vel_的第一位开始发布消息
        // for (size_t i = 0; i < vel_.size(); ++i) {
        //     std::cout << "Publishing_ " << i << std::endl;
        //     msg.data = vel_[i];
        //     pub.publish(msg);
        //     rate.sleep(); // 控制发布频率

        //     std::cout << "Published_flag " << should_publish << std::endl;

        //     // 检查是否需要终止当前遍历
        //     // {
        //     //     std::lock_guard<std::mutex> lock(vel_mutex); // 上锁
        //     //     if (stop_publishing) {
        //     //         break; // 终止当前遍历
        //     //     }
        //     // } // 解锁
        // }
    }
}
}
int main(int argc, char** argv) {
    ros::init(argc, argv, "publisher_node");
    ros::NodeHandle nh;

    // 创建用于发布消息的ROS发布者
    ros::Publisher pub = nh.advertise<std_msgs::Float64>("velocity", 1);

    // 初始化vel_
    vel_ = std::vector<double>(50, 0.0);

    // 启动发布消息的线程
    std::thread publish_thread(publishMessages, std::ref(pub), std::ref(nh));

    // 在主线程中更新vel_
    while (ros::ok()) {
        // 更新vel_，这里简单地将每个元素加1
        {
            //std::lock_guard<std::mutex> lock(vel_mutex); // 上锁
            for (auto& vel : vel_) {
                vel += 1.0;
            }
        } // 解锁

        // 通知发布线程立即开始从新的vel_的第一位开始发布
        {
            //std::lock_guard<std::mutex> lock_(stop_mutex); // 上锁
            ROS_WARN("Publish ################################################# ! ! ! !");
            update_flag = true;
        } // 解锁

        ROS_WARN("Solve Thread ! ! ! !");
        //cv.notify_one(); // 通知发布线程

        // 休眠一段时间，模拟更新频率
        ros::Duration(1.0).sleep();
    }

    // 等待发布线程结束
    publish_thread.join();

    return 0;
}