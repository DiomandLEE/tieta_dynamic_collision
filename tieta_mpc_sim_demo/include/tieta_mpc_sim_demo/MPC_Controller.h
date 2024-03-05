#ifndef MPC_CONTROLLER_H
#define MPC_CONTROLLER_H

#include <vector>
#include <map>
#include <Eigen/Core>
#include "ros/ros.h"
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

#include <geometry_msgs/TransformStamped.h>

//tf include
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/message_filter.h>

#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>

#include "tieta_mpc_sim_demo/Collision_Check.h"
#include "JointTrajPub/Angles.h"
#include "JointTrajPub/AnglesList.h"

using namespace std;

class MPC
{
    public:
        //构造函数
        MPC();
        MPC(Collision_Check &collision_check);

        Collision_Check _collision_check;

        // Solve the model given an initial state and ***.
        // Return the first actuatotions.
        vector<double> Solve(Eigen::VectorXd state, JointTrajPub::AnglesList trackTraj, vector<Eigen::Vector3d> tf_state, bool _terminal_flag, int _terminal_nums); //这个不需要Collision，因为已经成为公有变量了
        //用来存储solver计算的位置结果
        vector<double> mpc_x;
        vector<double> mpc_y;
        vector<double> mpc_theta;

        vector<double> mpc_joint1;
        vector<double> mpc_joint2;
        vector<double> mpc_joint3;
        vector<double> mpc_joint4;
        vector<double> mpc_joint5;
        vector<double> mpc_joint6;

        void LoadParams(const std::map<string, double> &params);

        double _mpc_total_cost;
        double _mpc_distx_Tcost;
        double _mpc_etheta_Tcost;
        double _mpc_disty_Tcost;
        double _mpc_joint1_Tcost;
        double _mpc_joint2_Tcost;
        double _mpc_joint3_Tcost;
        double _mpc_joint4_Tcost;
        double _mpc_joint5_Tcost;
        double _mpc_joint6_Tcost;

        ofstream file;
        ofstream file_debug;

        //纯数据,存放MPC预测时域的结果
        string _file_path_class_MPC;
        //带有文字介绍的,MPC预测时域的结果
        string _file_debug_path_class_MPC;
        //带有文字介绍的,跟踪的预测时域的轨迹
        string _file_debug_path_class_FG_eval;

    private:
        // Parameters for mpc solver
        // 在mec的车上，首先试验输入加速度a和角速度w(因为考虑到初始时刻和末尾时刻)
        double _max_angvel, _max_vel, _bound_value, _angel_upper, _angel_lower;
        //UR5e
        double _joint1_upper, _joint1_lower, _joint2_upper, _joint2_lower, _joint3_upper, _joint3_lower,
                _joint4_upper, _joint4_lower, _joint5_upper, _joint5_lower, _joint6_upper, _joint6_lower;
        // 这些是规定输入变量的顺序的start_index
        int _mpc_steps, _x_start, _y_start, _theta_start, _vx_start, _vy_start, _angvel_start;

        int _joint1_start, _joint2_start, _joint3_start, _joint4_start, _joint5_start, _joint6_start,
            _jntvel1_start, _jntvel2_start, _jntvel3_start, _jntvel4_start, _jntvel5_start, _jntvel6_start;

        std::map<string, double> _params;

        unsigned int _loop_cnt;
};

#endif /* MPC_CONTROLLER_H */