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

// #include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
// #include <ackermann_msgs/AckermannDriveStamped.h>
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
    void rcvtrajCB(const nav_msgs::Path::ConstPtr &pathMsg);
    void kinova_reach_init_callback(const std_msgs::Float32::ConstPtr &msg);
    bool get_kinova_reach_flag();
    double _time_start;

private:
    ros::NodeHandle _nh;
    ros::Subscriber _sub_timed_traj;
    ros::Publisher _pub_totalcost, _pub_distx_cost, _pub_etheta_cost, _pub_trackTraj, _pub_twist, _pub_mpctraj;
    tf::TransformListener _tf_listener;

    // geometry_msgs::Point _goal_pos;
    // nav_msgs::Odometry _odom;
    nav_msgs::Path _odom_path, _mpc_traj;

    ////处理后的给mpc_controll_loop执行的轨迹
    ////nav_msgs::Path _mpc_trackTraj;
    // flag pub & sub
    ros::Publisher _pub_initPose_reach;
    bool _mec_reach_init, _kinova_reach_init;
    std::mutex mlock;
    ros::Subscriber _sub_kinova_reach_init;

    // 发布cmd_vel
    geometry_msgs::Twist _twist_msg;

    string _globalPath_topic, _goal_topic;
    // vicon中的坐标系
    string _map_frame, _car_frame;

    // 判断是否到达第一个轨迹点的容差
    double _tolerence_xy, _tolerence_theta;

    MPC _mpc;
    map<string, double> _mpc_params;
    int _mpc_steps;
    double _w_distx, _w_disty, _w_etheta, _w_vel,
        _w_angvel, _w_acc, _w_angacc, _max_angvel, _bound_value, _angel_upper, _angel_lower;

    // double _Lf;
    // 机器人的运动信息
    double _dt, _angvel, _speed_x, _speed_y, _max_speed;

    // 计算Controller—Loop的次数，以更新待track的traj
    int _loop_count;
    // 需要轨迹中点的数量来确定，_loop_count的上限
    int _realTraj_length;
    // 中间变量，为了得到最终的traj中点的数量
    int _subTraj_length;
    int _controller_freq, _pid_freq, _thread_numbers;
    bool _traj_received, _track_finished, _trackTraj_computed,
        _tf_get, _pub_twist_flag, _debug_info, _delay_mode;

    // void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg);
    double range_angle_PI(double angle);
    double range_velocity_MAX(double _input_v);
    double range_angvel_MAX(double _input_w);
    nav_msgs::Path getTrackTraj(const nav_msgs::Path &rcvTrajMsg);

    // For generate trackTraj
    nav_msgs::Path _rcv_traj;

    // 每次计算得到的误差
    double _mpc_etheta;
    double _mpc_distx;
    double _mpc_disty;

    // 输出到file
    ofstream file;
    ofstream file_debug;

    string save_folder_path;
    string save_Debugfolder_path;

    // string save_Debugfile_path_MPC;
    // string save_Debugfile_path_FG_eval;

    // 计算耗时和时间起点
    double loop_duration;
    double _time_coordinate;

    // tf vicon中获取world和robot的坐标变换
    tf::TransformListener _tfListener;
    tf::StampedTransform _tfTransform;
    double _vicon_tf_x, _vicon_tf_y, _vicon_tf_theta;

    //定义vicon的转换矩阵
    Eigen::Matrix3d _vicon_tf_matrix;
    Eigen::Matrix3d _robot_tf_matrix;
    Eigen::Matrix3d _offset_matrix;

    // 机器人的状态
    double _rb_x, _rb_y, _rb_theta;
    double _last_rb_x, _last_rb_y, _last_rb_theta;

    //机器人map系下的速度,也是通过差分法计算得到的真实速度
    Eigen::Vector3d _vel_map;

    //机器人自身的cmd_vel速度
    Eigen::Vector3d _vel_bot;

    // goto InitState PID 参数
    // x方向的pid参数
    double _kp_vx, _ki_vx, _kd_vx;
    // y方向的pid参数
    double _kp_vy, _ki_vy, _kd_vy;
    // 角度的pid参数
    double _kp_omega, _ki_omega, _kd_omega;
}; // end of class

#endif /* MPC_Node_H */