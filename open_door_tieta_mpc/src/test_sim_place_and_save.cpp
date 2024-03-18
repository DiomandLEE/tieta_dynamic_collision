#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <urdf/model.h>
#include <iostream>
#include <fstream>
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
std::vector<double> current_velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

void velocityCallback(const std_msgs::Float64MultiArray::ConstPtr& velocity_msg) {
    // 使用速度更新关节位置
    std::lock_guard<std::mutex> lock(mutex);
    for (size_t i = 0; i < velocity_msg->data.size(); ++i) {
        current_velocity[i] = velocity_msg->data[i];
    }
}



bool start_door_flag = false;
//publish thread
void publish_thread_doorjoint_Velocity(ros::Publisher &pub, ros::NodeHandle &nh){
    std::string file_in = "/home/diamondlee/VKConTieta_ws/src/tieta_mpc_sim_demo/tieta_track_traj/place/retimed_placeTraj_15_36_51.csv";
    std::ifstream fs;
    fs.open(file_in);
    if (!fs.is_open())
    {
        ROS_ERROR("Cannot open file: %s", file_in.c_str());
        //return {};
    }
    std::vector<double> door_positions_vector;

    std::string lineStr;
    while (std::getline(fs, lineStr))
    {
        std::stringstream ss(lineStr);
        std::string item;
        std::vector<double> positions_vector;

        while (std::getline(ss, item, ','))
        {
            positions_vector.push_back(std::stod(item));
        }
        door_positions_vector.push_back(-1 * positions_vector[10]);
        //positions_vector.push_back(position_vector);
        //ROS_INFO("Publishing: %s", position_msg.data[0].c_str());
        //ROS_INFO("Publishing: %s", position_msg.data[1].c_str());
    }
    int rate_ = 10;
    ros::Rate rate(10);
    int i = 0;
    std::vector<double> joint_position = {0.0};
    while(ros::ok()){
        {
            while(!start_door_flag)
            {

            }
            sensor_msgs::JointState door_pos_msg;
            std::vector<std::string> joint_name = {"closet_bottom_right_door_joint"};
            joint_position[0] = door_positions_vector[i];

            pub.publish(door_pos_msg);

            i++;
            rate.sleep();
            ros::spinOnce();
        }
    }
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "test_joint_state_publisher");
    ros::NodeHandle n;

    ros::Publisher joint_state_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    ros::Subscriber joint_veloctiy_sub = n.subscribe("joint_velocity", 1, velocityCallback);

    ros::Rate loop_rate(pub_rate);

    //把这里做成param的参数，给到tieta的初始位置
    //std::vector<double> joint_positions = {-1.2,2.4,-5.99132e-10,1.57,-0.8678,-2.2043,-0.0347,1.6315,-4.37875e-11, -1.0};
    std::vector<double> joint_positions = {0.103956,0.953408,0.396687,1.84723,-0.0777131,-0.976665,0.77764,1.5597,-1.60804, 0.0, /*-0.1*/1.0, -5.5};
    // std::vector<std::string> joint_names = {"base_y_base_x", "base_theta_base_y", "base_link_base_theta", "right_arm_shoulder_pan_joint",
    //                                         "right_arm_shoulder_lift_joint", "right_arm_elbow_joint", "right_arm_wrist_1_joint",
    //                                         "right_arm_wrist_2_joint", "right_arm_wrist_3_joint", "dynamic_pedestrian_joint"};
    std::vector<std::string> joint_names = {"base_y_base_x", "base_theta_base_y", "base_link_base_theta", "right_arm_shoulder_pan_joint",
                                            "right_arm_shoulder_lift_joint", "right_arm_elbow_joint", "right_arm_wrist_1_joint",
                                            "right_arm_wrist_2_joint", "right_arm_wrist_3_joint", "closet_bottom_right_door_joint",
                                            "dynamic_pedestrian_joint_x", "dynamic_pedestrian_joint_y"};
    std::string file_in = "/home/diamondlee/VKConTieta_ws/src/tieta_mpc_sim_demo/tieta_track_traj/place/retimed_placeVelocity_15_36_51.csv";
    std::ifstream fs;
    fs.open(file_in);
    if (!fs.is_open())
    {
        ROS_ERROR("Cannot open file: %s", file_in.c_str());
        //return {};
    }
    std::vector<double> door_velocitys_vector;

    std::string lineStr;
    while (std::getline(fs, lineStr))
    {
        std::stringstream ss(lineStr);
        std::string item;
        std::vector<double> velocitys_vector;

        while (std::getline(ss, item, ','))
        {
            velocitys_vector.push_back(std::stod(item));
        }
        door_velocitys_vector.push_back(-1 * velocitys_vector[9]);
        //positions_vector.push_back(position_vector);
        //ROS_INFO("Publishing: %s", position_msg.data[0].c_str());
        //ROS_INFO("Publishing: %s", position_msg.data[1].c_str());
    }
    fs.close();
    std::string file_in1 = "/home/diamondlee/VKConTieta_ws/src/tieta_mpc_sim_demo/tieta_track_traj/place/retimed_placeTraj_15_36_51.csv";
    std::ifstream fs1;
    fs1.open(file_in1);
    if (!fs1.is_open())
    {
        ROS_ERROR("Cannot open file: %s", file_in.c_str());
        //return {};
    }
    std::vector<double> door_position_vector;

    std::string lineStr1;
    while (std::getline(fs1, lineStr1))
    {
        std::stringstream ss(lineStr1);
        std::string item;
        std::vector<double> position_vector;

        while (std::getline(ss, item, ','))
        {
            position_vector.push_back(std::stod(item));
        }
        door_position_vector.push_back(-1 * position_vector[10]);
        //positions_vector.push_back(position_vector);
        //ROS_INFO("Publishing: %s", position_msg.data[0].c_str());
        //ROS_INFO("Publishing: %s", position_msg.data[1].c_str());
    }
    fs1.close();

    auto _time_now = std::chrono::system_clock::now();
    auto _in_time_t = std::chrono::system_clock::to_time_t(_time_now);
    //使用put_time 格式化日期和身体的
    std::stringstream _folder_ss;
    _folder_ss << std::put_time(std::localtime(&_in_time_t), "%Y%m%d_%H%M%S");

    //构建文件夹的名称
    std::string
        start_foldername = "/home/diamondlee/VKConTieta_ws/src/open_door_tieta_mpc/positon_results";
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

    int door_velocity_num = 3;
    int door_position_num = 3;
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
            bool start_move = current_velocity[0] != 0.0 || current_velocity[1] != 0.0 || current_velocity[2] != 0.0 || current_velocity[3] != 0.0 ||
                                current_velocity[4] != 0.0 || current_velocity[5] != 0.0 || current_velocity[6] != 0.0 || current_velocity[7] != 0.0 ||
                                current_velocity[8] != 0.0;
            // bool start_move = true;
            if (start_move && (door_position_num < door_position_vector.size()))
            {
                //joint_positions[9] += door_velocitys_vector[door_velocity_num] * (1 / pub_rate) /10;
                joint_positions[9] = door_position_vector[door_position_num];
                if(num_ == 0)
                    file_out << joint_positions[0] << "," << joint_positions[1] << "," //<< joint_positions[2] << ","
                                << joint_positions[9] << "," << joint_positions[10] << "," << joint_positions[11] << std::endl;
                num_++;
                if(num_ >= 10)
                {
                    //door_velocity_num++;
                    file_out << joint_positions[0] << "," << joint_positions[1] << "," //<< joint_positions[2] << ","
                                // << joint_positions[3] << "," << joint_positions[4] << "," << joint_positions[5] << ","
                                // << joint_positions[6] << "," << joint_positions[7] << "," << joint_positions[8] << ","
                                << joint_positions[9] << "," << joint_positions[10] << "," << joint_positions[11] << std::endl;

                    door_position_num++;
                    num_ = 1;
                }
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


