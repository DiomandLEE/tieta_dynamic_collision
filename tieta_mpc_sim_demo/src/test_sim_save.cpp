#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <urdf/model.h>
#include <iostream>
#include <vector>
#include <std_msgs/Float64MultiArray.h>

//用于创建文件夹和获取当前的时间信息
#include <iomanip>
#include <chrono>
#include <ctime>
#include <boost/filesystem.hpp>

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
std::vector<double> current_velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

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

    auto _time_now = std::chrono::system_clock::now();
    auto _in_time_t = std::chrono::system_clock::to_time_t(_time_now);
    //使用put_time 格式化日期和身体的
    std::stringstream _folder_ss;
    _folder_ss << std::put_time(std::localtime(&_in_time_t), "%Y%m%d_%H%M%S");

    //构建文件夹的名称
    std::string
        start_foldername = "/home/diamondlee/VKConTieta_ws/src/tieta_mpc_sim_demo/position_results";
    const std::string foldername = start_foldername + "/mpc_position_" + _folder_ss.str();
    //创建文件夹,并检验文件夹是否创建
    if(!boost::filesystem::create_directory(foldername))
        ROS_ERROR("can`t creat the record folder: %s", foldername.c_str());
    else
        ROS_INFO("Create the mpc_positions_result record folder: %s", foldername.c_str());
    //创建CSV文件名
    const std::string record_filename = foldername + "/mpc_all_joints_positions.csv";
    //打开文件
    std::ofstream file_out;
    file_out = std::ofstream(record_filename);
    //file.open(record_filename,ios::app);
    if(file_out.is_open())
        ROS_INFO("mpc_resluts_positions file has been open !");
    else
        ROS_ERROR("Cannot open record file: %s", record_filename.c_str());

    ros::Publisher joint_state_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    ros::Subscriber joint_veloctiy_sub = n.subscribe("joint_velocity", 1, velocityCallback);

    ros::Rate loop_rate(pub_rate);
    // std::string urdf_file_name;
    // urdf::Model model;
    // if (n.getParam("robot_description", urdf_file_name)) {
    //     //std::cout << "urdf_file_name:" << urdf_file_name << std::endl;
    //     if (!model.initString(urdf_file_name)) {
    //         ROS_ERROR("Failed to parse urdf file");
    //         return -1;
    //     }
    // }
    // std::cout << "joint name:";
    // std::vector<urdf::JointSharedPtr> joints;
    // for (auto it = model.joints_.begin(); it != model.joints_.end(); it++) {
    //     urdf::JointSharedPtr joint = it->second;
    //     if (urdf::Joint::FIXED != joint->type) {
    //         joints.push_back(joint);
    //         std::cout << joint->name << " , ";
    //     }
    // }
    //把这里做成param的参数，给到tieta的初始位置
    std::vector<double> joint_positions = {-1.2,2.4,-5.99132e-10,1.57,-0.8678,-2.2043,-0.0347,1.6315,-4.37875e-11,-0.7,-0.3};
    std::vector<std::string> joint_names = {"base_y_base_x", "base_theta_base_y", "base_link_base_theta", "right_arm_shoulder_pan_joint",
                                            "right_arm_shoulder_lift_joint", "right_arm_elbow_joint", "right_arm_wrist_1_joint",
                                            "right_arm_wrist_2_joint", "right_arm_wrist_3_joint", /*"dynamic_pedestrian_joint"*/
                                            "dynamic_pedestrian_joint_x", "dynamic_pedestrian_joint_y"};
    int num_ = 0;
    // double position = 0.0;
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

            bool start_move = current_velocity[0] != 0.0 || current_velocity[1] != 0.0 || current_velocity[2] != 0.0 || current_velocity[3] != 0.0 ||
                                current_velocity[4] != 0.0 || current_velocity[5] != 0.0 || current_velocity[6] != 0.0 || current_velocity[7] != 0.0 ||
                                current_velocity[8] != 0.0;
            if (start_move)
            {
                //joint_positions[9] += door_velocitys_vector[door_velocity_num] * (1 / pub_rate) /10;
                //joint_positions[9] = door_position_vector[door_position_num];
                if(num_ == 0)
                    file_out << joint_positions[0] << "," << joint_positions[1] << "," << joint_positions[2] << ","
                                << joint_positions[3] << "," << joint_positions[4] << "," << joint_positions[5] << ","
                                << joint_positions[6] << "," << joint_positions[7] << "," << joint_positions[8] << ","
                                << joint_positions[9] << "," << joint_positions[10] << std::endl;
                num_++;
                if(num_ >= 10)
                {
                    //door_velocity_num++;
                    file_out << joint_positions[0] << "," << joint_positions[1] << "," << joint_positions[2] << ","
                                << joint_positions[3] << "," << joint_positions[4] << "," << joint_positions[5] << ","
                                << joint_positions[6] << "," << joint_positions[7] << "," << joint_positions[8] << ","
                                << joint_positions[9] << "," << joint_positions[10] << std::endl;

                    num_ = 1;
                }
            }
        }
        joint_state_pub.publish(joint_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
    }
