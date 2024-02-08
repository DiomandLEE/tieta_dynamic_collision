#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <urdf/model.h>
#include <iostream>
#include <vector>
#include <std_msgs/Float64MultiArray.h>

enum JointName
{
    base_y_base_x = 0,
    base_theta_base_y,
    base_link_base_theta,
    right_arm_shoulder_pan_joint,
    right_arm_shoulder_lift_joint,
    right_arm_elbow_joint,
    right_arm_wrist_1_joint,
    right_arm_wrist_2_joint,
    right_arm_wrist_3_joint,
    dynamic_pedestrian_joint
};

std::mutex mutex;

//    while (ros::ok()) {
//         sensor_msgs::JointState position;
//         {
//             std::lock_guard<std::mutex> lock(mutex);
//             position.position = current_position; 这个代码块我觉得挺妙的
//         }

//         position_pub.publish(position);

//         ros::spinOnce();
//         rate.sleep();
//     }
double pub_rate = 100.0;
std::vector<double> current_velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

void velocityCallback(const std_msgs::Float64MultiArray::ConstPtr& velocity_msg) {
    // 使用速度更新关节位置
    std::lock_guard<std::mutex> lock(mutex);
    for (size_t i = 0; i < velocity_msg->data.size(); ++i) {
        current_velocity[i] = velocity_msg->data[i];
    }
}

std::vector<double> updateSinglePosition(JointName joint_name, double velocity, const std::vector<double>& last_position) {
    std::vector<double> new_Position = last_position;

    switch (joint_name) {
        case base_y_base_x:
            new_Position[0] += velocity * 1 / pub_rate;
            break;

        case base_theta_base_y:
            new_Position[1] += velocity * 1 / pub_rate;
            break;

        case base_link_base_theta:
            new_Position[2] += velocity * 1 / pub_rate;
            break;

        case right_arm_shoulder_pan_joint:
            new_Position[3] += velocity * 1 / pub_rate;
            break;

        case right_arm_shoulder_lift_joint:
            new_Position[4] += velocity * 1 / pub_rate;
            break;

        case right_arm_elbow_joint:
            new_Position[5] += velocity * 1 / pub_rate;
            break;

        case right_arm_wrist_1_joint:
            new_Position[6] += velocity * 1 / pub_rate;
            break;

        case right_arm_wrist_2_joint:
            new_Position[7] += velocity * 1 / pub_rate;
            break;

        case right_arm_wrist_3_joint:
            new_Position[8] += velocity * 1 / pub_rate;
            break;

        case dynamic_pedestrian_joint:
            new_Position[9] += velocity * 1 / pub_rate;
            break;

            // Add more cases for other joint names

        default:
            //std::cout << "Invalid joint name." << std::endl;
            break;
    }

    return new_Position;
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "test_joint_state_publisher");
    ros::NodeHandle n;

    ros::Publisher joint_state_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    ros::Subscriber joint_veloctiy_sub = n.subscribe("joint_velocity", 1, velocityCallback);

    ros::Rate loop_rate(pub_rate);
    std::string urdf_file_name;
    urdf::Model model;
    if (n.getParam("robot_description", urdf_file_name)) {
        //std::cout << "urdf_file_name:" << urdf_file_name << std::endl;
        if (!model.initString(urdf_file_name)) {
            ROS_ERROR("Failed to parse urdf file");
            return -1;
        }
    }
    std::cout << "joint name:";
    std::vector<urdf::JointSharedPtr> joints;
    for (auto it = model.joints_.begin(); it != model.joints_.end(); it++) {
        urdf::JointSharedPtr joint = it->second;
        if (urdf::Joint::FIXED != joint->type) {
            joints.push_back(joint);
            std::cout << joint->name << " , ";
        }
    }
    //把这里做成param的参数，给到tieta的初始位置
    std::vector<double> joint_positions = {0.1, 0.0, 0.3, 0.7, 0.7, 0.0, 0.0, 0.0, 0.0, -3.0};
    std::vector<std::string> joint_names = {"base_y_base_x", "base_theta_base_y", "base_link_base_theta", "right_arm_shoulder_pan_joint",
                                            "right_arm_shoulder_lift_joint", "right_arm_elbow_joint", "right_arm_wrist_1_joint",
                                            "right_arm_wrist_2_joint", "right_arm_wrist_3_joint", "dynamic_pedestrian_joint"};

    //double position = 0.0;
    while (ros::ok()) {
        sensor_msgs::JointState joint_msg;
        joint_msg.header.stamp = ros::Time::now();
        //position = (position > 6.28) ? 0 : position + 0.01;
        joint_msg.name = joint_names;
        {
            std::lock_guard<std::mutex> lock(mutex);
            for (int i = 0; i < joint_positions.size(); i++) {
                joint_positions[i] += current_velocity[i] * (1 / pub_rate);
            }
            joint_msg.position = joint_positions; //这个代码块我觉得挺妙的
            joint_msg.velocity = current_velocity;
        }
        joint_state_pub.publish(joint_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
    }
    //! 需要/tf的，即在世界系下的坐标的：行人，底盘（但是这回发送的就是世界系下的速度了），///机械臂就是订阅joint_state就可以了
    //! 那些碰撞体就是需要一个Position，利用的是正向运动学
    //! 速度就是用差分来表示
    //todo 明天来搞定的就是正向运动学计算，计算ee的位置，计算机械臂的碰撞体的位置，对于底盘上的碰撞体，不需要正运动学，需要的是一个坐标转换矩阵到world系中。
    //! 这个cpp文件，就可以是一个单独的节点，用来接收mpc的输出，可视化，和发布机器人的状态。
/*
//用来做试验的，如何在多个joint中发布指定的joint信息
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_joint_state_publisher");
  ros::NodeHandle nh;

  ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 10);

  ros::Rate loop_rate(10); // 设置发布频率为10Hz
  int t = 0;

  while (ros::ok()) {
    sensor_msgs::JointState joint_state_msg;

    // 填充关节名称
    joint_state_msg.name.push_back("dynamic_pedestrian_joint");

    joint_state_msg.position = {0.0 + 0.1*0.1*t};
    joint_state_msg.effort = {};

    // 填充关节速度
    //joint_state_msg.velocity.push_back(0.1); // 设置joint3的速度为0.3 m/s

    // 设置时间戳为当前时间
    joint_state_msg.header.stamp = ros::Time::now();

    joint_state_msg.header.frame_id = "world"; // 设置坐标系ID

    // 发布关节状态消息
    joint_state_pub.publish(joint_state_msg);

    t++;

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
*/
/*
//试验读取所有的active joint，发现moveit还是得配置group
#include <ros/ros.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "get_active_joints");
  ros::NodeHandle nh;

  // 创建RobotModelLoader加载robot_description参数并构建RobotModel
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  const robot_model::RobotModelPtr& robot_model = robot_model_loader.getModel();

  // 获取RobotModel中的所有活动关节
  const robot_model::JointModelGroup* joint_model_group = robot_model->getJointModelGroup("manipulator");
  std::vector<std::string> active_joint_names = joint_model_group->getActiveJointModelNames();

  // 打印所有活动关节的名称
  ROS_INFO_STREAM("Active Joints:");
  for (const std::string& joint_name : active_joint_names) {
    ROS_INFO_STREAM("\t" << joint_name);
  }

  return 0;
}
*/
