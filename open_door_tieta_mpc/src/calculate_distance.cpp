#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <ros/ros.h>

#include <cmath>

#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/SVD>
#include <Eigen/QR>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/tree.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

Eigen::Vector3d door_axis, door_link_origin;
Eigen::Vector4d door_init_normal, door_bottom_tip_point, door_handle_point;



double calculate_project(KDL::Frame frame_ , Eigen::Vector3d door_bottom_)
{
    double proj_new_door_ = (-door_link_origin[0] + frame_.p.x()) * door_bottom_[0] +
                                (-door_link_origin[1] + frame_.p.y()) * door_bottom_[1] + (-door_link_origin[2] + frame_.p.z()) * door_bottom_[2];
    //proj_new_door的模
    double mod_door_bottom_ = std::sqrt(std::pow(door_bottom_[0], 2)) +
                                                std::pow(door_bottom_[1], 2) +
                                                    std::pow(door_bottom_[2], 2);
    proj_new_door_ = proj_new_door_ / mod_door_bottom_;
    return proj_new_door_;
}

double calculate_normal_dist(KDL::Frame frame_ , Eigen::Vector3d door_normal_world)
{
    double dist_new_door_ = (-door_link_origin[0] + frame_.p.x()) * door_normal_world[0] +
                                (-door_link_origin[1] + frame_.p.y()) * door_normal_world[1] + (-door_link_origin[2] + frame_.p.z()) * door_normal_world[2];
    return dist_new_door_;
}

double calculate_door_handle(KDL::Frame frame_ , KDL::Frame door_handle_)
{
    double dist_handle = std::sqrt(std::pow((-door_handle_.p.x() + frame_.p.x()), 2) +
                                        std::pow((-door_handle_.p.y() + frame_.p.y()), 2) +
                                            std::pow((-door_handle_.p.z() + frame_.p.z()), 2));
    return dist_handle;
}
//door_left_bottom
double calculate_door_bottom(KDL::Frame frame_ , Eigen::Vector3d door_left_bottom_)
{
    double dist_bottom = std::sqrt(std::pow((-door_left_bottom_[0] + frame_.p.x()), 2) +
                                        std::pow((-door_left_bottom_[1] + frame_.p.y()), 2));
    return dist_bottom;
}

double calculate_ped_distance(KDL::Frame frame_ , double x_, double y_)
{
    double dist_ped = std::sqrt(std::pow((-x_ + frame_.p.x()), 2) +
                                        std::pow((-y_ + frame_.p.y()), 2));
    return dist_ped;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "double_csv_merger");
    ros::NodeHandle nh;

    std::string finaldata_folder = "/home/diamondlee/VKConTieta_ws/src/open_door_tieta_mpc/positon_results/mpc_position_2024";
    std::string mpc_file_time = "0319_193553";
    std::string mpc_file_suffix = "/mpc_all_joints_positions.csv";

    std::string ik_result_folder = "/home/diamondlee/VKConTieta_ws/src/open_door_tieta_mpc/positon_results/ik_position_2024";
    std::string ik_file_time = "0319_195039";
    std::string ik_file_suffix = "/ik_theta_URjoints_positions.csv";

    std::string data_folder = "/home/diamondlee/VKConTieta_ws/src/open_door_tieta_mpc/distance_results/";
    std::string start_direction = "right_up";

    nh.param<std::string>("/calculate_distance_ik/mpc_file_time", mpc_file_time, "");
    nh.param<std::string>("/calculate_distance_ik/ik_file_time", ik_file_time, "0321_120331");
    nh.param<std::string>("/calculate_distance_ik/start_direction", start_direction, "");

    std::string finaldata_file = finaldata_folder + mpc_file_time + mpc_file_suffix;
    ROS_WARN("finaldata_file: %s", finaldata_file.c_str());
    std::string ik_result_file = ik_result_folder + ik_file_time + ik_file_suffix;
    ROS_WARN("ik_result_file: %s", ik_result_file.c_str());

    std::string pedestrian_data_file = data_folder + start_direction + "_pedestrian.csv";
    ROS_WARN("pedestrian_data_file: %s", pedestrian_data_file.c_str());
    std::ofstream file_ped(pedestrian_data_file);

    std::string door_data_file = data_folder + start_direction + "_door.csv";
    ROS_WARN("door_data_file: %s", door_data_file.c_str());
    std::ofstream file_door(door_data_file);

    std::string handle_data_file = data_folder + start_direction + "_handle.csv";
    ROS_WARN("handle_data_file: %s", handle_data_file.c_str());
    std::ofstream file_handle(handle_data_file);

    double base_threshold, shoulder_threshold, elbow_threshold, wrist_threshold, gripper_threshold;
    nh.param<double>("/calculate_distance_ik/base_threshold", base_threshold, 0.0);
    nh.param<double>("/calculate_distance_ik/shoulder_threshold", shoulder_threshold, 0.0);
    nh.param<double>("/calculate_distance_ik/elbow_threshold", elbow_threshold, 0.0);
    nh.param<double>("/calculate_distance_ik/wrist_threshold", wrist_threshold, 0.0);
    nh.param<double>("/calculate_distance_ik/gripper_threshold", gripper_threshold, 0.0);

    //柜门打开的角度
    door_axis << 0, 0, 1;

    door_init_normal << 1, 0, 0, 1;
    door_bottom_tip_point << 0, -0.56, 0, 1;
    door_handle_point << 0.0105, -0.507, 1.051498720329, 1;
    door_link_origin << 1.4755, 0.0605, 0.085;

    // 门的长度:0.56
    //正运动学
    std::string urdf_file = "/home/diamondlee/VKConTieta_ws/src/urdf_description/robot_description/ridgeback_dual_arm_description/urdf/vkc_big_task_place_door.urdf";

    // 解析URDF文件，并创建KDL树
    KDL::Tree kdl_tree;
    if (!kdl_parser::treeFromFile(urdf_file, kdl_tree))
    {
        ROS_ERROR("Failed to construct KDL tree");
        return -1;
    }

    //获取world和sphere_collision在树中的链（shoulder）：
    KDL::Chain chain_shoulder;
    std::string base_link = "world";
    std::string tip_link_shoulder = "right_arm_shoulder_sphere_link";

    // 获取从base_link到tip_link_shoulder的链
    if (!kdl_tree.getChain(base_link, tip_link_shoulder, chain_shoulder))
    {
        ROS_ERROR("Failed to get KDL chain world-to-shoulder!");
        return -1;
    }

    //获取world和sphere_collision在树中的链（elbow）：
    KDL::Chain chain_elbow;
    std::string tip_link_elbow = "right_arm_forearm_sphere_link";

    // 获取从base_link到tip_link_elbow的链
    if (!kdl_tree.getChain(base_link, tip_link_elbow, chain_elbow))
    {
        ROS_ERROR("Failed to get KDL chain world-to-elbow!");
        return -1;
    }

    //获取world和sphere_collision在树中的链（wrist）：
    KDL::Chain chain_wrist;
    std::string tip_link_wrist = "right_arm_wrist_sphere_link";

    // 获取从base_link到tip_link_wrist的链
    if (!kdl_tree.getChain(base_link, tip_link_wrist, chain_wrist))
    {
        ROS_ERROR("Failed to get KDL chain world-to-wrist!");
        return -1;
    }

    //获取world和sphere_collision在树中的链（gripper）：
    KDL::Chain chain_gripper;
    std::string tip_link_gripper = "right_arm_flange_gripper_sphere";

    // 获取从base_link到tip_link_gripper的链
    if (!kdl_tree.getChain(base_link, tip_link_gripper, chain_gripper))
    {
        ROS_ERROR("Failed to get KDL chain world-to-gripper!");
        return -1;
    }


    KDL::Chain chain_arm_base;
    std::string tip_link_arm_base = "right_arm_shoulder_link";

    if(!kdl_tree.getChain(base_link, tip_link_arm_base, chain_arm_base))
    {
        ROS_ERROR("Failed to get KDL chain world-to-arm_base!");
    }

    KDL::Chain chain_handle;
    std::string tip_link_handle = "closet_bottom_right_handle";
    if(!kdl_tree.getChain(base_link, tip_link_handle, chain_handle))
    {
        ROS_ERROR("Failed to get KDL chain world-to-handle!");
    }

    KDL::Chain chain_door_bottom;
    std::string tip_link_door_bottom = "door_lfet_bottom";
    if(!kdl_tree.getChain(base_link, tip_link_door_bottom, chain_door_bottom))
    {
        ROS_ERROR("Failed to get KDL chain world-to-door_bottom!");
    }
    //! 请确保在URDF文件中正确指定了base_link和tip_link的名称。

    //创建KDL正向运动学求解器，计算sphere_collision_link的坐标和姿态角：
    // 定义正向运动学求解器
    KDL::ChainFkSolverPos_recursive fk_solver_shoulder(chain_shoulder);
    KDL::ChainFkSolverPos_recursive fk_solver_elbow(chain_elbow);
    KDL::ChainFkSolverPos_recursive fk_solver_wrist(chain_wrist);
    KDL::ChainFkSolverPos_recursive fk_solver_gripper(chain_gripper);
    KDL::ChainFkSolverPos_recursive fk_solver_arm_base(chain_arm_base);
    KDL::ChainFkSolverPos_recursive fk_solver_handle(chain_handle);
    KDL::ChainFkSolverPos_recursive fk_solver_door_bottom(chain_door_bottom);

    //segment就是link
    // 定义关节角
    KDL::JntArray joint_positions_shoulder(chain_shoulder.getNrOfJoints());
    KDL::JntArray joint_positions_elbow(chain_elbow.getNrOfJoints());
    KDL::JntArray joint_positions_wrist(chain_wrist.getNrOfJoints());
    KDL::JntArray joint_positions_gripper(chain_gripper.getNrOfJoints());
    KDL::JntArray joint_positions_arm_base(chain_arm_base.getNrOfJoints());
    KDL::JntArray joint_positions_handle(chain_handle.getNrOfJoints());
    KDL::JntArray joint_positions_door_bottom(chain_door_bottom.getNrOfJoints());
    std::cout << "arm_base_joints_size: " << joint_positions_arm_base.data.size() << std::endl;
    std::cout << "handle_joints_size: " << joint_positions_handle.data.size() << std::endl;
    std::cout << "door_bottom_joints_size: " << joint_positions_door_bottom.data.size() << std::endl;
    // << 0 << std::endl;

    // 读取第一个CSV文件的数据并存储在向量中
    std::ifstream file_in_mpc(finaldata_file);
    std::vector<std::vector<double>> data_mpc;
    std::string line;
    while (std::getline(file_in_mpc, line)) {
        std::vector<double> row;
        std::istringstream iss(line);
        std::string val;
        while (std::getline(iss, val, ',')) {
            row.push_back(std::stod(val));
        }
        // std::cout << "row: " << row.size() << std::endl;
        // //data_mpc.push_back(row);
        // std::cout << row[10] << std::endl;
        // std::cout << row[11] << std::endl;
        row[3] = row[3] -0.1;
        row[4] = row[4] + 4.5;
        data_mpc.push_back(row);
    }
    file_in_mpc.close();

    // 读取第二个CSV文件的数据并存储在向量中
    std::ifstream file_in_ik(ik_result_file);
    std::vector<std::vector<double>> data_ik;
    while (std::getline(file_in_ik, line)) {
        std::vector<double> row;
        std::istringstream iss(line);
        std::string val;
        while (std::getline(iss, val, ',')) {
            row.push_back(std::stod(val));
        }
        data_ik.push_back(row);
    }
    file_in_ik.close();

    // 合并数据并存储在一个新的向量中
    std::vector<std::vector<double>> FinalData;
    for (size_t i = 0; i < data_mpc.size(); ++i) {
        std::vector<double> FinalRow;
        // 添加第一个文件的前两列
        FinalRow.push_back(data_mpc[i][0]);
        FinalRow.push_back(data_mpc[i][1]);
        // 添加第二个文件的所有列
        FinalRow.insert(FinalRow.end(), data_ik[i].begin(), data_ik[i].end());
        FinalData.push_back(FinalRow);
    }

    // 添加第一个文件剩余的数据
    for (size_t i = 0; i < data_mpc.size(); ++i) {
        std::vector<double> remainingRow(data_mpc[i].begin() + 2, data_mpc[i].end());
        FinalData[i].insert(FinalData[i].end(), remainingRow.begin(), remainingRow.end());
    }
    std::cout << "FinalData size: " << FinalData.size() << std::endl;
    std::cout << "FinalData[0] size: " << FinalData[0].size() << std::endl;
    std::cout << FinalData[0][0] << "," << FinalData[0][1]
                << "," << FinalData[0][2] << "," << FinalData[0][3]
            << "," << FinalData[0][4] << "," << FinalData[0][5]
                            << "," << FinalData[0][6] << "," << FinalData[0][7]
                                            << "," << FinalData[0][8] << "," << FinalData[0][9]
                                            << "," << FinalData[0][10] << "," << FinalData[0][11] << std::endl;

    for(auto finaldata : FinalData)
    {
        //read to pedestain
        //base
        KDL::Frame frame_base;
        frame_base.p[0] = finaldata[0];
        frame_base.p[1] = finaldata[1];
        frame_base.p[2] = 0.0;

        file_ped << calculate_ped_distance(frame_base, finaldata[10], finaldata[11]) << ",";
        // << 1 << std::endl;

        //shoulder
        for(unsigned int i=0; i < joint_positions_shoulder.data.size(); i++)
        {
            joint_positions_shoulder.data[i] = finaldata[i];
        }
        // 计算正向运动学，并获取结果
        KDL::Frame frame_shoulder;
        fk_solver_shoulder.JntToCart(joint_positions_shoulder, frame_shoulder);

        file_ped << calculate_ped_distance(frame_shoulder, finaldata[10], finaldata[11]) << ",";
        // << 2 << std::endl;


        for(unsigned int i=0; i < joint_positions_elbow.data.size(); i++)
        {
            joint_positions_elbow.data[i] = finaldata[i];
        }
        KDL::Frame frame_elbow;
        fk_solver_elbow.JntToCart(joint_positions_elbow, frame_elbow);
        //elbow

        file_ped << calculate_ped_distance(frame_elbow, finaldata[10], finaldata[11]) << ",";
        // << 3 << std::endl;


        for(unsigned int i=0; i < joint_positions_wrist.data.size(); i++)
        {
            joint_positions_wrist.data[i] = finaldata[i];
        }
        KDL::Frame frame_wrist;
        fk_solver_wrist.JntToCart(joint_positions_wrist, frame_wrist);
        //
        file_ped << calculate_ped_distance(frame_wrist, finaldata[10], finaldata[11]) << ",";
        // << 4 << std::endl;

        for(unsigned int i=0; i < joint_positions_gripper.data.size(); i++)
        {
            joint_positions_gripper.data[i] = finaldata[i];
        }
        KDL::Frame frame_gripper;
        fk_solver_gripper.JntToCart(joint_positions_gripper, frame_gripper);

        file_ped << calculate_ped_distance(frame_gripper, finaldata[10], finaldata[11]) << std::endl;
        // << 4 << std::endl;

        //READ 计算到handle的距离
        for(unsigned int i=0; i < joint_positions_handle.data.size(); i++)
        {
            joint_positions_handle.data[i] = finaldata[9]; //它这个应该是只有门的旋转关节这一个active joint
        }
        KDL::Frame frame_handle;
        fk_solver_handle.JntToCart(joint_positions_handle, frame_handle);

        // << 5 << std::endl;

        for(unsigned int i=0; i < joint_positions_arm_base.data.size(); i++)
        {
            joint_positions_arm_base.data[i] = finaldata[i];
        }
        KDL::Frame frame_arm_base;
        fk_solver_arm_base.JntToCart(joint_positions_arm_base, frame_arm_base);

        file_handle << calculate_door_handle(frame_arm_base, frame_handle) << std::endl;
        // << 6 << std::endl;

        //READ 计算到door的距离
        //首先，计算门的法向量
        double door_angle = finaldata[9];
        Eigen::AngleAxisd rotation_(door_angle, door_axis);
        Eigen::Matrix3d door2new_door = rotation_.toRotationMatrix();
        //Eigen::Matrix3d new_door2door_rotate = door2new_door.transpose(); // 转置也可以，因为正交矩阵，转置和逆等同。
        // 平移向量是
        Eigen::Vector3d new_door_trans = Eigen::Vector3d::Zero();
        // 构造齐次矩阵
        Eigen::Matrix4d new_door2door = Eigen::Matrix4d::Identity();
        new_door2door.block<3, 3>(0, 0) = door2new_door;
        new_door2door.block<3, 1>(0, 3) = new_door_trans;


        double angle = M_PI; // 绕z轴旋转180度，即π弧度
        Eigen::Matrix3d door2world_rotate;
        door2world_rotate << -1, 0, 0,
                            0, -1, 0,
                            0, 0, 1;

        // 构造齐次变换矩阵
        Eigen::Matrix4d door2world = Eigen::Matrix4d::Identity();
        door2world.block<3, 3>(0, 0) = door2world_rotate;
        door2world.block<3, 1>(0, 3) = door_link_origin;

        //!当然可以要求机器人必须在门前面，（如果是在🚪的正前方的话，靠投影来判断）
        //!此外，对于法向量，是需要两个端点相减的，或者不考虑旋转矩阵的平移，因为向量不是固定的，就表示一个方向
        Eigen::Matrix4d door2world_for_normal = Eigen::Matrix4d::Identity();
        door2world_for_normal.block<3, 3>(0, 0) = door2world_rotate;
        // 平移向量是全0
        door2world_for_normal.block<3, 1>(0, 3) = Eigen::Vector3d::Zero();

        Eigen::Matrix4d new_door2world = door2world * new_door2door;
        Eigen::Matrix4d new_door2world_for_normal = door2world_for_normal * new_door2door;
        Eigen::Vector3d door_normal_world = (new_door2world_for_normal * door_init_normal).block<3, 1>(0, 0);
        Eigen::Vector3d door_bottom_tip_point_world = (new_door2world * door_bottom_tip_point).block<3, 1>(0, 0);
        // std::cout << "door_normal_world" << door_normal_world << std::endl;
        // std::cout << "door_bottom_tip_point_world" << door_bottom_tip_point_world << std::endl;
        Eigen::Vector3d door_bottom_ = door_bottom_tip_point_world - door_link_origin;
        //std::cout << "door_bottom_" << door_bottom_ << std::endl;

        //base
        //std::cout << "base" << calculate_project(frame_base,door_bottom_) << std::endl;
        if(calculate_project(frame_base,door_bottom_) < 0.56 + base_threshold)
            file_door << calculate_normal_dist(frame_base, door_normal_world) << ",";
        else
            file_door << calculate_door_bottom(frame_base,door_bottom_tip_point_world) << ",";

        //shoulder
        if(calculate_project(frame_shoulder,door_bottom_) < 0.56 + shoulder_threshold)
            file_door << calculate_normal_dist(frame_shoulder, door_normal_world) << ",";
        else
            file_door << calculate_door_bottom(frame_shoulder,door_bottom_tip_point_world) << ",";

        //elbow
        if(calculate_project(frame_elbow,door_bottom_) < 0.56 + elbow_threshold)
            file_door << calculate_normal_dist(frame_elbow, door_normal_world) << ",";
        else
            file_door << calculate_door_bottom(frame_elbow,door_bottom_tip_point_world) << ",";

        //wrist
        if(calculate_project(frame_wrist,door_bottom_) < 0.56 + wrist_threshold)
            file_door << calculate_normal_dist(frame_wrist, door_normal_world) << ",";
        else
            file_door << calculate_door_bottom(frame_wrist,door_bottom_tip_point_world) << ",";

        //gripper
        if(calculate_project(frame_gripper,door_bottom_) < 0.56 + gripper_threshold)
            file_door << calculate_normal_dist(frame_gripper, door_normal_world) << std::endl;
        else
            file_door << calculate_door_bottom(frame_gripper, door_bottom_tip_point_world) << std::endl;
    }

    // << "New CSV file 'mpc_ik_data.csv' created successfully in the specified folder.\n";

    return 0;
}