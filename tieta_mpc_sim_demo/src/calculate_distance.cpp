#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <ros/ros.h>
#include <cmath>

#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/SVD>
#include <eigen3/Eigen/QR>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/tree.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

int main(int argc, char **argv) {
    ros::init(argc, argv, "csv_merger");
    ros::NodeHandle nh;

    std::string mpc_result_folder = "/home/diamondlee/VKConTieta_ws/src/tieta_mpc_sim_demo/position_results//mpc_position_2024";
    std::string file_time = "0318_173803";
    std::string file_suffix = "/mpc_all_joints_positions.csv";

    std::string data_folder = "/home/diamondlee/VKConTieta_ws/src/tieta_mpc_sim_demo/distance_results/";
    std::string start_direction = "horizontal_left";

    nh.param<std::string>("/calculate_distance/file_time", file_time, "0318_173803");
    nh.param<std::string>("/calculate_distance/start_direction", start_direction, "horizontal_left");

    std::string mpc_result_file = mpc_result_folder + file_time + file_suffix;
    ROS_WARN("mpc_result_file: %s", mpc_result_file.c_str());

    std::string pedestrian_data_file = data_folder + start_direction + "_" + file_time + "_pedestrian.csv";
    ROS_WARN("pedestrian_data_file: %s", pedestrian_data_file.c_str());
    std::ofstream file_ped(pedestrian_data_file);
    std::string closet_data_file = data_folder + start_direction + "_" + file_time + "_closet.csv";
    ROS_WARN("closet_data_file: %s", closet_data_file.c_str());
    std::ofstream file_clo(closet_data_file);

    Eigen::Vector3d closet_point;
    Eigen::Vector3d closet_front_normal;
    closet_point << 1.4755, 0.03, 0.085;
    closet_front_normal << -1, 0, 0;

    // 读取第一个CSV文件的数据并存储在向量中
    std::ifstream file_in(mpc_result_file);
    std::vector<std::vector<double>> mpc_results;
    std::string line;
    while (std::getline(file_in, line)) {
        std::vector<double> row;
        std::istringstream iss(line);
        std::string val;
        while (std::getline(iss, val, ',')) {
            row.push_back(std::stod(val));
        }
        mpc_results.push_back(row);
    }
    file_in.close();

    //正运动学
    std::string urdf_file = "/home/diamondlee/VKConTieta_ws/src/urdf_description/robot_description/ridgeback_dual_arm_description/urdf/vkc_big_task.urdf.urdf";

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

    //!请确保在URDF文件中正确指定了base_link和tip_link的名称。

    //创建KDL正向运动学求解器，计算sphere_collision_link的坐标和姿态角：
    // 定义正向运动学求解器
    KDL::ChainFkSolverPos_recursive fk_solver_shoulder(chain_shoulder);
    KDL::ChainFkSolverPos_recursive fk_solver_elbow(chain_elbow);
    KDL::ChainFkSolverPos_recursive fk_solver_wrist(chain_wrist);
    KDL::ChainFkSolverPos_recursive fk_solver_gripper(chain_gripper);


    //segment就是link
    // 定义关节角
    KDL::JntArray joint_positions_shoulder(chain_shoulder.getNrOfJoints());
    KDL::JntArray joint_positions_elbow(chain_elbow.getNrOfJoints());
    KDL::JntArray joint_positions_wrist(chain_wrist.getNrOfJoints());
    KDL::JntArray joint_positions_gripper(chain_gripper.getNrOfJoints());

    for(auto mpc_result : mpc_results)
    {
        //base
        file_ped << std::sqrt(std::pow(mpc_result[0] - mpc_result[9], 2)
                                                + std::pow(mpc_result[1] - mpc_result[10], 2)) << ",";
        file_clo << (-closet_point[0] + mpc_result[0]) * closet_front_normal[0] +
                                (-closet_point[1] + mpc_result[1]) * closet_front_normal[1] +
                                                (-closet_point[2] + 0.0) * closet_front_normal[2] << ",";

        for(unsigned int i=0; i < joint_positions_shoulder.data.size(); i++)
        {
            joint_positions_shoulder.data[i] = mpc_result[i];
            //joint_positions_shoulder.data[i] = 0.0;
        }
        // 计算正向运动学，并获取结果
        KDL::Frame frame_shoulder;
        fk_solver_shoulder.JntToCart(joint_positions_shoulder, frame_shoulder);
        //shoulder
        file_ped << std::sqrt(std::pow(frame_shoulder.p.x() - mpc_result[9], 2)
                                                + std::pow(frame_shoulder.p.y() - mpc_result[10], 2)) << ",";
        file_clo << (-closet_point[0] + frame_shoulder.p.x()) * closet_front_normal[0] +
                                (-closet_point[1] + frame_shoulder.p.y()) * closet_front_normal[1] +
                                                (-closet_point[2] + frame_shoulder.p.z()) * closet_front_normal[2] << ",";

        for(unsigned int i=0; i < joint_positions_elbow.data.size(); i++)
        {
            joint_positions_elbow.data[i] = mpc_result[i];
        }
        KDL::Frame frame_elbow;
        fk_solver_elbow.JntToCart(joint_positions_elbow, frame_elbow);
        //elbow
        file_ped << std::sqrt(std::pow(frame_elbow.p.x() - mpc_result[9], 2)
                                                + std::pow(frame_elbow.p.y() - mpc_result[10], 2)) << ",";
        file_clo << (-closet_point[0] + frame_elbow.p.x()) * closet_front_normal[0] +
                                (-closet_point[1] + frame_elbow.p.y()) * closet_front_normal[1] +
                                                (-closet_point[2] + frame_elbow.p.z()) * closet_front_normal[2] << ",";

        for(unsigned int i=0; i < joint_positions_wrist.data.size(); i++)
        {
            joint_positions_wrist.data[i] = mpc_result[i];
        }
        KDL::Frame frame_wrist;
        fk_solver_wrist.JntToCart(joint_positions_wrist, frame_wrist);
        //wrist
        file_ped << std::sqrt(std::pow(frame_wrist.p.x() - mpc_result[9], 2)
                                                + std::pow(frame_wrist.p.y() - mpc_result[10], 2)) << ",";
        file_clo << (-closet_point[0] + frame_wrist.p.x()) * closet_front_normal[0] +
                                (-closet_point[1] + frame_wrist.p.y()) * closet_front_normal[1] +
                                                (-closet_point[2] + frame_wrist.p.z()) * closet_front_normal[2] << ",";

        for(unsigned int i=0; i < joint_positions_gripper.data.size(); i++)
        {
            joint_positions_gripper.data[i] = mpc_result[i];
        }
        KDL::Frame frame_gripper;
        fk_solver_gripper.JntToCart(joint_positions_gripper, frame_gripper);
        //gripper
        file_ped << std::sqrt(std::pow(frame_gripper.p.x() - mpc_result[9], 2)
                                                + std::pow(frame_gripper.p.y() - mpc_result[10], 2)) << std::endl;
        file_clo << (-closet_point[0] + frame_gripper.p.x()) * closet_front_normal[0] +
                                (-closet_point[1] + frame_gripper.p.y()) * closet_front_normal[1]
                                                + (-closet_point[2] + frame_gripper.p.z()) * closet_front_normal[2] << std::endl;
    }


    // 将合并后的数据写入新的CSV文件
    file_ped.close();
    file_clo.close();

    std::cout << "New CSV file 'data.csv' created successfully in the specified folder.\n";

    return 0;
}