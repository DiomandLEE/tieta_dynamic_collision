#ifndef COLLISION_CHECK_H
#define COLLISION_CHECK_H

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>

#include <cmath>
#include <iostream>
#include <vector>
#include <string>

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
#include <mutex>
#include <NLsolver/cppad/ipopt/solve.hpp>
typedef CPPAD_TESTVECTOR(double) Dvector;
using CppAD::AD;
class Collision_Check
{
private:
    /* data */
    std::string urdf_file;

    std::string base_link = "world";
    std::string tip_link_shoulder = "right_arm_shoulder_sphere_link";
    std::string tip_link_elbow = "right_arm_forearm_sphere_link";
    std::string tip_link_wrist = "right_arm_wrist_sphere_link";
    std::string tip_link_gripper = "right_arm_flange_gripper_sphere";
    std::string ee_tool_link = "right_arm_tool0";

    KDL::Tree kdl_tree;
    KDL::Chain kdl_chain_shoulder;
    KDL::Chain kdl_chain_elbow;
    KDL::Chain kdl_chain_wrist;
    KDL::Chain kdl_chain_gripper;
    KDL::Chain kdl_chain_ee_tool;

    KDL::JntArray joint_positions_shoulder;
    KDL::JntArray joint_positions_elbow;
    KDL::JntArray joint_positions_wrist;
    KDL::JntArray joint_positions_gripper;
    KDL::JntArray joint_positions_ee_tool;

    KDL::Frame frame_shoulder;
    KDL::Frame frame_elbow;
    KDL::Frame frame_wrist;
    KDL::Frame frame_gripper;
    KDL::Frame frame_ee_tool;

    //!这里没有默认构造函数，那么就用每次构造+析构的方式
    // KDL::ChainFkSolverPos_recursive fk_solver_shoulder;
    // KDL::ChainFkSolverPos_recursive fk_solver_elbow;
    // KDL::ChainFkSolverPos_recursive fk_solver_wrist;
    // KDL::ChainFkSolverPos_recursive fk_solver_gripper;

    Eigen::Matrix3d R_robot_world;
    Eigen::Vector3d t_robot_world;

    Eigen::Vector3d sphere_lf_robot;
    Eigen::Vector3d sphere_rf_robot;
    Eigen::Vector3d sphere_lr_robot;
    Eigen::Vector3d sphere_rr_robot;

    Eigen::Vector3d sphere_lf_world;
    Eigen::Vector3d sphere_rf_world;
    Eigen::Vector3d sphere_lr_world;
    Eigen::Vector3d sphere_rr_world;

    std::vector<Dvector> collision_points;

public:
    Collision_Check(/* args */);
    //初始化列表
    Collision_Check(const std::string robot_description):urdf_file(robot_description){
        //解析urdf
        if (!kdl_parser::treeFromFile(urdf_file, kdl_tree))
        {
            ROS_ERROR("Failed to construct KDL tree");
        }
        //从Tree解析出每一个chain
        if (!kdl_tree.getChain(base_link, tip_link_shoulder, kdl_chain_shoulder))
        {
            ROS_ERROR("Failed to get KDL chain world-to-shoulder!");
        }
        if (!kdl_tree.getChain(base_link, tip_link_elbow, kdl_chain_elbow))
        {
            ROS_ERROR("Failed to get KDL chain world-to-elbow!");
        }
        if (!kdl_tree.getChain(base_link, tip_link_wrist, kdl_chain_wrist))
        {
            ROS_ERROR("Failed to get KDL chain world-to-wrist!");
        }
        if (!kdl_tree.getChain(base_link, tip_link_gripper, kdl_chain_gripper))
        {
            ROS_ERROR("Failed to get KDL chain world-to-gripper!");
        }
        if (!kdl_tree.getChain(base_link, ee_tool_link, kdl_chain_ee_tool))
        {
            ROS_ERROR("Failed to get KDL chain world-to-ee_tool!");
        }
        //计算正运动学的KDL::JntArray
        joint_positions_shoulder = KDL::JntArray(kdl_chain_shoulder.getNrOfJoints());
        joint_positions_elbow = KDL::JntArray(kdl_chain_elbow.getNrOfJoints());
        joint_positions_wrist = KDL::JntArray(kdl_chain_wrist.getNrOfJoints());
        joint_positions_gripper = KDL::JntArray(kdl_chain_gripper.getNrOfJoints());
        joint_positions_ee_tool = KDL::JntArray(kdl_chain_ee_tool.getNrOfJoints());
        sphere_lf_robot << 0.2, 0.15, 0.15;
        sphere_rf_robot << 0.2, -0.15, 0.15;
        sphere_lr_robot << -0.2, 0.15, 0.15;
        sphere_rr_robot << -0.2, -0.15, 0.15;
    }
    //计算，各个Sphere Position
    void calculate_sphere_position(const std::vector<double> currentJointPositions);
    //获取，每一个Spere Position
    std::vector<Dvector> get_sphere_position();

    ~Collision_Check();
};

#endif // COLLISION_CHECK_HPP
