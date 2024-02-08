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

std::mutex jointStateMutex;
sensor_msgs::JointState currentJointState;
std::vector<double> currentJointPositions = {};
std::vector<double> currentJointVelocities = {};

void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    //std::cout << "Joint state1 received!" << std::endl;
    //std::lock_guard<std::mutex> lock(jointStateMutex);
    //std::cout << "Joint state2 received!" << std::endl;
    // 存储关节位置和关节速度
    currentJointState = *msg;
    //std::cout << "Joint state3 received!" << std::endl;
}

//right_arm_forearm_sphere_link: elbow
//right_arm_shoulder_sphere_link: shoulder
//right_arm_wrist_sphere_link: wrist
//right_arm_flange_gripper_sphere: gripper
int main(int argc, char** argv) {
    //在代码中解析URDF文件以及创建KDL树（KDL Tree）：
    // 创建ROS节点
    ros::init(argc, argv, "kdl_tree_example");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<sensor_msgs::JointState>("joint_states", 1, jointStateCallback);
    //currentJointState.position = {0.1, 0.0, 0.3, 0.7, 0.7, 0.0, 0.0, 0.0, 0.0, -3.0};
    ros::AsyncSpinner spinner(3); // Use multi threads
    spinner.start();
    // 通过ROS参数服务器获取URDF文件路径
    ros::Duration(1.0).sleep(); // Wait for joint state to be received
    std::string urdf_file = "/home/diamondlee/VKConTieta_ws/src/urdf_description/robot_description/ridgeback_dual_arm_description/urdf/vkc_big_task.urdf.urdf";
    // nh.getParam("urdf_file", urdf_file);

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
    //############################### 检查关节名称 #######################################
    // 获取关节名称
    std::vector<std::string> joint_names;
    for (unsigned int i = 0; i < chain_shoulder.getNrOfSegments(); ++i) {
        const KDL::Segment& segment = chain_shoulder.getSegment(i);
        joint_names.push_back(segment.getJoint().getName());
    }

    // 打印关节名称
    for (const std::string& joint_name : joint_names) {
        std::cout << "Joint Name: " << joint_name << std::endl;
    }
    std::cout << "Joint Number: " << chain_shoulder.getNrOfJoints() << std::endl;


    //############################# 检查active joint名称 #################################
    std::vector<std::string> activeJointNames;

    // //获取active joint，这个不对
    // for (unsigned int i = 0; i < chain_shoulder.getNrOfJoints(); ++i) {
    //     // 如果关节没有被禁用（active），则获取关节名称
    //     if (chain_shoulder.getSegment(i).getJoint().getDisabled() == false) {
    //         std::string jointName = chain_shoulder.getSegment(i).getJoint().getName();
    //         activeJointNames.push_back(jointName);
    //     }
    // }
    for (unsigned int i = 0; i < chain_shoulder.getNrOfSegments(); ++i) {
        const KDL::Segment& segment = chain_shoulder.getSegment(i);
        if(segment.getJoint().getType() != KDL::Joint::None)
            activeJointNames.push_back(segment.getJoint().getName());
        //joint_names.push_back(segment.getJoint().getName());
    }

    // 打印活动关节的名称
    for (const auto& name : activeJointNames) {
        std::cout << "Active Joint Name: " << name << std::endl;
    }

    {
        std::lock_guard<std::mutex> lock(jointStateMutex);

    }
    // 定义关节角
    KDL::JntArray joint_positions_shoulder(chain_shoulder.getNrOfJoints());
    for(unsigned int i=0; i < joint_positions_shoulder.data.size(); i++)
    {
        // std::cout << "debug" <<
        // std::endl;
        //std::lock_guard<std::mutex> lock(jointStateMutex);
        // std::cout << "debug1" <<
        // std::endl;
        std::cout << "size: " << joint_positions_shoulder.data.size() << std::endl;
        std::cout << "currentJointState.position.size(): " << currentJointState.position.size() << std::endl;
        std::cout << "currentJointState.position[i]: " << currentJointState.position[i] << std::endl;
        joint_positions_shoulder.data[i] = currentJointState.position[i];
        //joint_positions_shoulder.data[i] = 0.0;
    }

    // 设置关节角frame_shoulder

    // 计算正向运动学，并获取结果
    KDL::Frame frame_shoulder;
    fk_solver_shoulder.JntToCart(joint_positions_shoulder, frame_shoulder);

    // 获取坐标和姿态角
    // double x = frame.p.x();
    // double y = frame.p.y();
    // double z = frame.p.z();
    double roll, pitch, yaw;
    frame_shoulder.M.GetRPY(roll, pitch, yaw);
    std::cout << "x: " << frame_shoulder.p.x() << std::endl;
    std::cout << "y: " << frame_shoulder.p.y() << std::endl;
    std::cout << "z: " << frame_shoulder.p.z() << std::endl;
    std::cout << "roll: " << roll << std::endl;
    std::cout << "pitch: " << pitch << std::endl;
    std::cout << "yaw: " << yaw << std::endl;

    tf::TransformListener listener;
    tf::StampedTransform transform;
    try {
        listener.waitForTransform("/world", "/right_arm_shoulder_sphere_link", ros::Time(0), ros::Duration(1.0));
        listener.lookupTransform("/world", "/right_arm_shoulder_sphere_link", ros::Time(0), transform);
    } catch (tf::TransformException& ex) {
        ROS_ERROR("%s", ex.what());
    }
    std::cout << "tf_x: " << transform.getOrigin().x() << std::endl;
    std::cout << "tf_y: " << transform.getOrigin().y() << std::endl;
    std::cout << "tf_z: " << transform.getOrigin().z() << std::endl;
    double tf_roll, tf_pitch, tf_yaw;
    tf::Matrix3x3 rotation(transform.getRotation());
    rotation.getRPY(tf_roll, tf_pitch, tf_yaw);
    std::cout << "tf_roll: " << tf_roll << std::endl;
    std::cout << "tf_pitch: " << tf_pitch << std::endl;
    std::cout << "tf_yaw: " << tf_yaw << std::endl;
    // ros::Rate rate(10.0);
    // while (nh.ok()) {
    //     tf::StampedTransform transform;
    //     try {
    //         listener.lookupTransform("/world", "/base_link", ros::Time(0), transform);
    //     } catch (tf::TransformException& ex) {
    //         ROS_ERROR("%s", ex.what());
    //         ros::Duration(1.0).sleep();
    //         continue;
    //     }

    //     tf::Matrix3x3 rotation(transform.getRotation());
    //     tf::Vector3 translation(transform.getOrigin());

    //     tf::Matrix4x4 transformMatrix;
    //     rotation.getRotation(transformMatrix);
    //     transformMatrix.setOrigin(translation);

    //     ROS_INFO_STREAM("Transform Matrix: \n" << transformMatrix);

    //     rate.sleep();
    // }


    //tf,先验证一下

    ros::waitForShutdown();

    //!ros::spin();
}

    //最后，你可以使用x、y、z、roll、pitch和yaw来获取sphere_collision_link的坐标和姿态角。