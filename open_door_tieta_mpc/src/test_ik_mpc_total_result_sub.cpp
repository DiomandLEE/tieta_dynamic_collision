#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <urdf/model.h>
#include <iostream>
#include <vector>
#include <std_msgs/Float64MultiArray.h>

std::mutex mutex;

double pub_rate = 100.0;

std::vector<double> current_positions(12, 0.0);

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
    ros::init(argc, argv, "ikmpc_csv_sub_node");
    ros::NodeHandle nh;
    ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
    ros::Subscriber position_sub = nh.subscribe("csv_ikmpc_pos", 1, positionCallback);
    ros::Rate loop_rate(pub_rate);

    std::vector<double> joint_positions = {0.103956,0.953408,0.396687,1.84723,-0.0777131,-0.976665,0.77764,1.5597,-1.60804, 0.0, /*-0.1*/1.2, -6.3};
    current_positions = {0.103956,0.953408,0.396687,1.84723,-0.0777131,-0.976665,0.77764,1.5597,-1.60804, 0.0, /*-0.1*/1.2, -6.3};
    std::vector<std::string> joint_names = {"base_y_base_x", "base_theta_base_y", "base_link_base_theta", "right_arm_shoulder_pan_joint",
                                            "right_arm_shoulder_lift_joint", "right_arm_elbow_joint", "right_arm_wrist_1_joint",
                                            "right_arm_wrist_2_joint", "right_arm_wrist_3_joint", "closet_bottom_right_door_joint",
                                            "dynamic_pedestrian_joint_x", "dynamic_pedestrian_joint_y"};

    while (ros::ok()) {
        sensor_msgs::JointState joint_msg;
        joint_msg.header.stamp = ros::Time::now();
        //position = (position > 6.28) ? 0 : position + 0.01;
        joint_msg.name = joint_names;
        {
            std::lock_guard<std::mutex> lock(mutex);
            for (int i = 0; i < current_positions.size(); i++) {
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