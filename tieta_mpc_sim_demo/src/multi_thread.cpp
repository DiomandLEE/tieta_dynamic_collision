#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <thread>
#include <chrono>

class ComputationNode {
public:
    ComputationNode()
    : nh("~"), publish_rate(1000) {  // Set a high publish rate
        publisher = nh.advertise<std_msgs::Float64>("high_freq_topic", 10);
    }

    void start() {
        // Start the publishing thread
        std::thread pub_thread(&ComputationNode::publishLoop, this);

        // Main loop with heavy calculation
        ros::Rate loop_rate(1);  // Assuming heavy calculation runs at 1Hz
        while (ros::ok()) {
            performHeavyCalculation();//!在这里执行，繁重的计算
            ros::spinOnce();  // Handle ROS events
            loop_rate.sleep();
        }

        pub_thread.join();  // Join the thread before exiting
    }

private:
    ros::NodeHandle nh;
    ros::Publisher publisher;
    double publish_rate;
    double value = 0.0;  // Example value to publish

    void publishLoop() {
        ros::Rate loop_rate(publish_rate);
        while (ros::ok()) {
            std_msgs::Float64 msg;
            msg.data = value;  // Assign the value to publish
            publisher.publish(msg);
            loop_rate.sleep();
        }
    }

    void performHeavyCalculation() {
        // Placeholder for heavy computation
        // This is where you'd implement your heavy computation logic
        // For demonstration, we'll just increment the value
        value += 1.0;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "computation_node");
    ComputationNode node;
    node.start();
    return 0;
}