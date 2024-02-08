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

using namespace std;

class MPC
{
    public:
        //构造函数
        MPC();

        // Solve the model given an initial state and ***.
        // Return the first actuatotions.
        // vector<double> Solve(Eigen::VectorXd state, nav_msgs::Path trackTraj, int loop_cnt);
        vector<double> Solve(Eigen::VectorXd state, nav_msgs::Path trackTraj);
        vector<double> mpc_x;
        vector<double> mpc_y;
        vector<double> mpc_theta;

        void LoadParams(const std::map<string, double> &params);

        double _mpc_total_cost;
        double _mpc_distx_Tcost;
        double _mpc_etheta_Tcost;
        double _mpc_disty_Tcost;

        double _mpc_acc_x_Tcost;
        double _mpc_acc_y_Tcost;
        double _mpc_angacc_Tcost;

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
        // 这些是规定输入变量的顺序的start_index
        int _mpc_steps, _x_start, _y_start, _theta_start, _vx_start, _vy_start, _angvel_start;
        std::map<string, double> _params;
        unsigned int _loop_cnt;

};

#endif /* MPC_CONTROLLER_H */