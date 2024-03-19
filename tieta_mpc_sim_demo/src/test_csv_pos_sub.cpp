#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <urdf/model.h>
#include <iostream>
#include <vector>
#include <std_msgs/Float64MultiArray.h>

std::mutex mutex;

double pub_rate = 100.0;

std::vector<double> current_positions(11, 0.0);

void positionCallback(const std_msgs::Float64MultiArray::ConstPtr& position_msg){
    //使用Position更新关节的位置
    std::lock_guard<std::mutex> lock(mutex);
    for(int i=0; i<position_msg->data.size(); i++)
    {
        current_positions[i] = position_msg->data[i];
        //std::cout << current_positions[i] << ",";
    }
    //std::cout << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_csv_sub_node");
    ros::NodeHandle nh;
    ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
    ros::Subscriber position_sub = nh.subscribe("csv_test_pos", 1, positionCallback);
    ros::Rate loop_rate(pub_rate);

    std::vector<double> joint_positions = {-1.2, 2.4, -5.99132e-10, 1.57, -0.8678, -2.2043, -0.0347, 1.6315, -4.37875e-11, 0.8, -8.2};
    current_positions = joint_positions;
    std::vector<std::string> joint_names = {"base_y_base_x", "base_theta_base_y", "base_link_base_theta", "right_arm_shoulder_pan_joint",
                                            "right_arm_shoulder_lift_joint", "right_arm_elbow_joint", "right_arm_wrist_1_joint",
                                            "right_arm_wrist_2_joint", "right_arm_wrist_3_joint", /*"dynamic_pedestrian_joint"*/
                                            "dynamic_pedestrian_joint_x", "dynamic_pedestrian_joint_y"};
    while (ros::ok()) {
        sensor_msgs::JointState joint_msg;
        joint_msg.header.stamp = ros::Time::now();
        //position = (position > 6.28) ? 0 : position + 0.01;
        joint_msg.name = joint_names;
        {
            std::lock_guard<std::mutex> lock(mutex);
            for (int i = 0; i < joint_positions.size(); i++) {
                joint_positions[i] = current_positions[i];
            }
            //joint_positions[9] = -3.0;
            joint_msg.position = joint_positions; // 这个代码块我觉得挺妙的
        }
        joint_state_pub.publish(joint_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}