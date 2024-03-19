#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/frames_io.hpp>
#include <iostream>
#include <fstream>
#include <vector>

#include <kdl/chainiksolverpos_lma.hpp>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/tree.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <ros/ros.h>

//用于创建文件夹和获取当前的时间信息
#include <iomanip>
#include <chrono>
#include <ctime>
#include <boost/filesystem.hpp>

std::vector<double> parseDoorTrajCsv(ros::NodeHandle nh)
{
    std::string file_in1 = "/home/diamondlee/VKConTieta_ws/src/tieta_mpc_sim_demo/tieta_track_traj/place/retimed_placeTraj_15_36_51.csv";
    std::ifstream fs1;
    fs1.open(file_in1);
    if (!fs1.is_open())
    {
        ROS_ERROR("Cannot open file: %s", file_in1.c_str());
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
    return door_position_vector;
}

std::vector<std::vector<double>> parseMPCresultCsv(ros::NodeHandle nh)
{
    //!mpc_0312_170736
    //!mpc_0319_160600
    //!mpc_0319_174902
    //!mpc_0319_181831
    //!mpc_0319_185509
    //!mpc_0319_193553
    std::string file_in1 = "/home/diamondlee/VKConTieta_ws/src/open_door_tieta_mpc/positon_results/mpc_position_20240319_193553/mpc_all_joints_positions.csv";
    std::ifstream fs1;
    fs1.open(file_in1);
    if (!fs1.is_open())
    {
        ROS_ERROR("Cannot open file: %s", file_in1.c_str());
        //return {};
    }
    std::vector<std::vector<double>> mpc_result_vector;

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
        mpc_result_vector.push_back(position_vector);
        //positions_vector.push_back(position_vector);
        //ROS_INFO("Publishing: %s", position_msg.data[0].c_str());
        //ROS_INFO("Publishing: %s", position_msg.data[1].c_str());
    }
    return mpc_result_vector;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ur5_kdl_ik");
    ros::NodeHandle nh;

    auto _time_now = std::chrono::system_clock::now();
    auto _in_time_t = std::chrono::system_clock::to_time_t(_time_now);
    //使用put_time 格式化日期和身体的
    std::stringstream _folder_ss;
    _folder_ss << std::put_time(std::localtime(&_in_time_t), "%Y%m%d_%H%M%S");

    //构建文件夹的名称
    std::string
        start_foldername = "/home/diamondlee/VKConTieta_ws/src/open_door_tieta_mpc/positon_results";
    const std::string foldername = start_foldername + "/ik_position_" + _folder_ss.str();
    //创建文件夹,并检验文件夹是否创建
    if(!boost::filesystem::create_directory(foldername))
        ROS_ERROR("can`t creat the record folder: %s", foldername.c_str());
    else
        ROS_INFO("Create the ik_result record folder: %s", foldername.c_str());
    //创建CSV文件名
    const std::string record_filename = foldername + "/ik_theta_URjoints_positions.csv";
    //打开文件
    std::ofstream file_out;
    file_out = std::ofstream(record_filename);
    //file.open(record_filename,ios::app);
    if(file_out.is_open())
        ROS_INFO("ik_positions file has been open !");
    else
        ROS_ERROR("Cannot open ik file: %s", record_filename.c_str());

    std::vector<double> door_position_vector = parseDoorTrajCsv(nh);
    std::vector<std::vector<double>> mpc_result_vector = parseMPCresultCsv(nh);

    //READ 首先，利用场景open_door，获取handle的世界系下的坐标
    std::string urdf_file_closet_without_vkc;
    urdf_file_closet_without_vkc = "/home/diamondlee/VKConTieta_ws/src/urdf_description/robot_description/ridgeback_dual_arm_description/urdf/vkc_big_task_place_door_urdf.urdf";

    std::string base_link_ = "world";
    std::string tip_link_ = "closet_bottom_right_handle";

    KDL::Tree kdl_tree_;
    KDL::Chain kdl_chain_closet;
    KDL::JntArray q_door_;

    //解析urdf
    if (!kdl_parser::treeFromFile(urdf_file_closet_without_vkc, kdl_tree_))
    {
        ROS_ERROR("Failed to construct KDL tree");
    }

    //从Tree解析出每一个chain
    if (!kdl_tree_.getChain(base_link_, tip_link_, kdl_chain_closet))
    {
        ROS_ERROR("Failed to get KDL chain world-to-closet_bottom_right_handle!");
    }

    //计算正运动学的KDL::JntArray
    q_door_ = KDL::JntArray(kdl_chain_closet.getNrOfJoints());
    ROS_WARN("world_ to Door_Handle active joint number is : %d", kdl_chain_closet.getNrOfJoints());

    /*
    //加入循环
    for(unsigned int i = 0; i < q_door_.data.size(); i++)
    {
        q_door_(i) = door_position_vector[i]; //todo这里要换成csv中的joint_door
    }

    KDL::ChainFkSolverPos_recursive fk_solver_handle_(kdl_chain_closet);
    KDL::Frame frame_handle;

    fk_solver_handle_.JntToCart(q_door_, frame_handle);
    ROS_ERROR("closer_bottom_right_handle XYZRPY in World has been computed !!!");
    */

    std::string base_link_1 = "world";
    std::string tip_link_1 = "virtual_base_theta";

    KDL::Tree kdl_tree_1;
    KDL::Chain kdl_chain_base;
    KDL::JntArray q_base_xy;

    //从Tree解析出每一个chain
    if (!kdl_tree_.getChain(base_link_1, tip_link_1, kdl_chain_base))
    {
        ROS_ERROR("Failed to get KDL chain world-to-base!");
    }

    //计算正运动学的KDL::JntArray
    q_base_xy = KDL::JntArray(kdl_chain_base.getNrOfJoints());
    ROS_WARN("world_ to base_theta active joint number is : %d", kdl_chain_base.getNrOfJoints());

    /*
    //加入循环
    for(unsigned int i = 0; i < q_base_xy.data.size(); i++)
    {
        q_base_xy(i) = 0.0; //todo这里要换成csv中的joint_door
    }

    KDL::ChainFkSolverPos_recursive fk_solver_base_(kdl_chain_base);
    KDL::Frame frame_base;

    fk_solver_base_.JntToCart(q_base_xy, frame_base);
    ROS_ERROR("virtual_base_theta XYZRPY in World has been computed !!!");
    */

    // 构建base_theta到closet_bottom_right_handle的KDL链
    KDL::Chain kdl_chain_vkc;
    KDL::Tree kdl_tree_vkc;

    std::string urdf_file_closet_vkc;
    urdf_file_closet_vkc = "/home/diamondlee/VKConTieta_ws/src/open_door_tieta_mpc/urdf/vkc_door.urdf";

    std::string base_link_vkc = "virtual_base_theta";
    std::string tip_link_vkc = "closet_bottom_right_handle";

    //解析urdf
    if (!kdl_parser::treeFromFile(urdf_file_closet_vkc, kdl_tree_vkc))
    {
        ROS_ERROR("Failed to construct KDL tree VKC");
    }

    //从Tree解析出每一个chain
    if (!kdl_tree_vkc.getChain(base_link_vkc, tip_link_vkc, kdl_chain_vkc))
    {
        ROS_ERROR("Failed to get KDL chain world-to-closet_bottom_right_handle!");
    }

    // 构建正向运动学求解器
    KDL::ChainFkSolverPos_recursive fksolver_vkc(kdl_chain_vkc);

    // 构建逆运动学求解器
    KDL::ChainIkSolverVel_pinv iksolverv_vkc(kdl_chain_vkc);
    KDL::ChainIkSolverPos_NR iksolver_vkc(kdl_chain_vkc, fksolver_vkc, iksolverv_vkc);

    // 设置目标末端姿态
    ROS_WARN("virtual_base_theta_ to Door_Handle active joint number is : %d", kdl_chain_vkc.getNrOfJoints());
    KDL::JntArray jointpositions(kdl_chain_vkc.getNrOfJoints());
    // 在这里设置末端的位置和姿态

    /*
    //放入循环
    // 初始化所有关节位置
    for (unsigned int i = 0; i < kdl_chain_vkc.getNrOfJoints(); i++) {
        jointpositions(i) = 0.0;
    }

    // // 设置第一个关节（索引为0）的位置为固定值，例如：90度（弧度制）
    // jointpositions(0) = M_PI / 2.0; // 90度转换为弧度

    // 进行逆运动学求解

    KDL::Vector pos(frame_handle.p.x() - frame_base.p.x(), frame_handle.p.y() - frame_base.p.y(),
                            frame_handle.p.z() - frame_base.p.z());
    KDL::Frame handle_in_base(frame_handle.M, pos);

    int ret = iksolver_vkc.CartToJnt(jointpositions, handle_in_base, jointpositions);

    if (ret >= 0) {
        // 输出逆运动学求解结果
        std::cout << "Joint positions after IK:" << std::endl;
        for (unsigned int i = 0; i < kdl_chain_vkc.getNrOfJoints(); i++) {
            std::cout << jointpositions(i) << " ";
        }
        std::cout << std::endl;
    } else {
        std::cerr << "Failed to solve IK" << std::endl;
    }
    */
    int num_count = 0;
    KDL::JntArray q_init_(7);
    //theta+6joints
    q_init_.data << 0.396687, 1.84723, -0.0777131, -0.976665, 0.77764, 1.5597, -1.60804;
    for (int num_res_ = 0; num_res_ < mpc_result_vector.size(); num_res_++)
    {
        //READ 计算handle在世界系下的位置
        for(unsigned int i = 0; i < q_door_.data.size(); i++)
        {
            q_door_(i) = mpc_result_vector[num_res_][2]; //todo这里要换成csv中的joint_door
        }

        KDL::ChainFkSolverPos_recursive fk_solver_handle_(kdl_chain_closet);
        KDL::Frame frame_handle;

        fk_solver_handle_.JntToCart(q_door_, frame_handle);
        //ROS_ERROR("closer_bottom_right_handle XYZRPY in World has been computed !!!");

        //READ 计算base_theta在world系下的位置
        for(unsigned int i = 0; i < q_base_xy.data.size(); i++)
        {
            q_base_xy(i) = mpc_result_vector[num_res_][i];
        }

        KDL::ChainFkSolverPos_recursive fk_solver_base_(kdl_chain_base);
        KDL::Frame frame_base;

        fk_solver_base_.JntToCart(q_base_xy, frame_base);

        //READ 计算逆运动学
        for (unsigned int i = 0; i < kdl_chain_vkc.getNrOfJoints(); i++) {
            jointpositions(i) = q_init_(i);
        }
        //!mpc_0312_170736
        //观察csv文件一共多少行，看看需要旋转多少度，平均一下
        // jointpositions(0) = jointpositions(0) + 0.0013;
        //!mpc_0319_160600
        // jointpositions(0) = jointpositions(0) - 0.0010;
        //!mpc_0319_174902
        //jointpositions(0) = jointpositions(0) + 0.004;
        //!mpc_0319_181831
        //jointpositions(0) = jointpositions(0) - 0.0015;
        //!mpc_0319_185509
        //jointpositions(0) = jointpositions(0) + 0.004;
        //!mpc_0319_193553
        jointpositions(0) = jointpositions(0) - 0.004;
        KDL::JntArray last_jntpos = jointpositions;


        KDL::Vector pos(frame_handle.p.x() - frame_base.p.x(), frame_handle.p.y() - frame_base.p.y(),
                                frame_handle.p.z() - frame_base.p.z());
        KDL::Frame handle_in_base(frame_handle.M, pos);

        int ret = iksolver_vkc.CartToJnt(jointpositions, handle_in_base, jointpositions);

        if (ret >= 0) {
            // 输出逆运动学求解结果
            //std::cout << "Joint positions after IK:" << std::endl;
            for (unsigned int i = 0; i < kdl_chain_vkc.getNrOfJoints() - 1; i++) {
                file_out << jointpositions(i) << ",";
            }
            file_out << jointpositions(kdl_chain_vkc.getNrOfJoints() - 1) << std::endl;
            q_init_ = jointpositions;
        } else {
            num_count++;
            for (unsigned int i = 0; i < kdl_chain_vkc.getNrOfJoints(); i++) {
                file_out << last_jntpos(i) << ",";
            }
            file_out << 12345678 << std::endl;
            q_init_ = last_jntpos;
            //std::cerr << "Failed to solve IK" << std::endl;
        }
    }

    ROS_WARN("Failed to solve IK for %d counts", num_count);
    return 0;
}