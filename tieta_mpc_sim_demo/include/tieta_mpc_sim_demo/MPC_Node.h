#ifndef MPC_Node_H
#define MPC_Node_H

#include <iostream>
#include <map>
#include <math.h>

#include "ros/ros.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>

// tf include
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/message_filter.h>

#include <std_msgs/Float32.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

#include "tieta_mpc_sim_demo/FG_evalue.h"
#include "tieta_mpc_sim_demo/MPC_Controller.h"
#include "tieta_mpc_sim_demo/Collision_Check.h"
#include <Eigen/Core>
#include <Eigen/QR>
#include <Eigen/Dense>

// inlcude iostream and string libraries
#include <iostream>
#include <fstream>
#include <string>

#include <thread>
#include <mutex>

//用于创建文件夹和获取当前的时间信息
#include <iomanip>
#include <chrono>
#include <ctime>
#include <boost/filesystem.hpp>

#include <sensor_msgs/JointState.h>
#include <urdf/model.h>
#include <iostream>
#include <vector>
#include <std_msgs/Float64MultiArray.h>

#include "JointTrajPub/Angles.h"
#include "JointTrajPub/AnglesList.h"


using namespace std;
//namespace fs = std::filesystem;

class MPCNode
{
public:
    MPCNode(); // 进入构造函数
    ~MPCNode();
    int get_thread_numbers();
    int get_controll_freq();
    bool controlLoop();
    bool gotoInitState();
    void rcvJointTrajCB(const JointTrajPub::AnglesListConstPtr &totalTrajMsg);
    void getRobotStateCB(const sensor_msgs::JointStateConstPtr &robotstateMsg);
    void setJointVelocity(const std::vector<double> &jointVelocity); //这个里面是要包含行人速度的
    double _time_start;

private:
    ros::NodeHandle _nh;
    ros::Subscriber _sub_timed_traj, _sub_robot_state;
    //ros::Publisher _pub_totalcost, _pub_distx_cost, _pub_etheta_cost, _pub_trackTraj, _pub_twist, _pub_mpctraj;
    ros::Publisher _pub_robot_velocity;

    //tf作用仅仅是监听当前的行人状态,以及获取每次control的起始障碍物位置。
    tf::TransformListener _tf_listener;

    //! Collision Check类
    Collision_Check _collision_check;
    std::string _robot_description;

    // geometry_msgs::Point _goal_pos;
    // nav_msgs::Odometry _odom;
    JointTrajPub::AnglesList _odom_path, _mpc_traj;

    // flag pub & sub
    ros::Publisher _pub_initPose_reach;

    std::mutex mlock; //锁，用来保护rviz发来的机器人的状态不重复使用

    // 发布cmd_vel
    std_msgs::Float64MultiArray _jntvel_msg;

    // 判断是否到达第一个轨迹点的容差
    double _tolerence_xy, _tolerence_theta, _tolerence_joint;

    MPC _mpc;
    map<string, double> _mpc_params;
    int _mpc_steps;
    //碰撞安全阈值
    double _base_threshold, _shoulder_threshold, _elbow_threshold, _wrist_threshold, _gripper_threshold;
    //动态障碍物阈值
    double _pedestrian_threshold, _pedestrian_vel;
    //对于EE的姿态限制
    double _tool_x,_tool_y,_tool_z,_tool_roll,_tool_pitch,_tool_yaw;

    double _w_distx, _w_disty, _w_etheta, _w_vel,_w_angvel,
            _w_jnt, _w_jntvel,
            _w_base_collision,_w_shoulder_collision, _w_elbow_collision, _w_wrist_collision, _w_gripper_collision, _w_hard_EE_tool,
            _bound_value, _angel_upper, _angel_lower;

    double _joint1_upper, _joint1_lower, _joint2_upper, _joint2_lower, _joint3_upper, _joint3_lower, _joint4_upper, _joint4_lower,
            _joint5_upper, _joint5_lower, _joint6_upper, _joint6_lower;

    // double _Lf;
    // 机器人的运动信息
    std::vector<double> _joint_pos, _joint_vel;
    double _dt, _angvel, _speed_x, _speed_y, _max_speed, _max_angvel;
    double _jntvel1, _jntvel2, _jntvel3, _jntvel4, _jntvel5, _jntvel6, _max_jntvel;
    Eigen::Vector3d dynamic_pedestrian_pos, base_lf_sphere_pos, base_rf_sphere_pos, base_lr_sphere_pos, base_rr_sphere_pos,
                        shoulder_sphere_pos, elbow_sphere_pos, wrist_sphere_pos, gripper_sphere_pos;
    std::vector<Eigen::Vector3d> _tf_state;


    // 计算Controller—Loop的次数，以更新待track的traj
    int _loop_count;
    // 需要轨迹中点的数量来确定，_loop_count的上限
    int _realTraj_length;
    // 中间变量，为了得到最终的traj中点的数量
    int _subTraj_length;

    int _controller_freq, _pid_freq, _thread_numbers;

    bool _traj_received, _track_finished, _trackTraj_computed,
        _sensor_get, _debug_info, _delay_mode;//_sensor_get

    // void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg);
    double range_angle_PI(double angle);
    double range_velocity_MAX(double _input_v);
    double range_angvel_MAX(double _input_w);
    double range_jntvel_MAX(double _input_w);

    JointTrajPub::AnglesList getTrackTraj(const JointTrajPub::AnglesList &rcvJointTrajMsg);

    // For generate trackTraj
    JointTrajPub::AnglesList _rcv_traj;

    // 每次计算得到的误差
    double _mpc_etheta;
    double _mpc_distx;
    double _mpc_disty;

    // 输出到file
    ofstream file;
    ofstream file_debug;

    string save_folder_path;
    string save_Debugfolder_path;

    // 计算耗时和时间起点
    double loop_duration;
    double _time_coordinate;

    // tf vicon中获取world和robot的坐标变换
    tf::TransformListener _tfListener;
    tf::StampedTransform _tfTransform;

    // goto InitState PID 参数
    // x方向的pid参数
    double _kp_vx, _ki_vx, _kd_vx;
    // y方向的pid参数
    double _kp_vy, _ki_vy, _kd_vy;
    // 角度的pid参数
    double _kp_omega, _ki_omega, _kd_omega;
}; // end of class

#endif /* MPC_Node_H */