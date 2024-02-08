#ifndef FG_evalue_H
#define FG_evalue_H

#include <NLsolver/cppad/ipopt/solve.hpp>
#include <Eigen/Core>
#include "ros/ros.h"
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

#include <geometry_msgs/TransformStamped.h>

//tf include
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/message_filter.h>

#include <fstream>
#include <iostream>
#include <string>
#include <iomanip>

#include "tieta_mpc_sim_demo/Collision_Check.h"

using namespace std;
using CppAD::AD;

class FG_eval
{
    public:
        //构造函数
        FG_eval(nav_msgs::Path _trackTraj);

        //加载public成员的参数
        void LoadParams(const std::map<string, double> &params);

        //定义目标函数F和约束信息函数G  MPC implementation (cost func & constraints)
        typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
        // fg: function that evaluates the objective and constraints using the syntax
        void operator()(ADvector &fg, const ADvector &vars);

        //Eigen::VectorXd coeffs;
        double _dt;
        double _w_distx,_w_disty, _w_etheta, _w_vel, _w_angvel, _w_acc, _w_angacc;
        int _mpc_steps, _x_start, _y_start, _theta_start, _vx_start, _vy_start, _angvel_start;
        //ros::Subscriber _mpc_trackTraj_sub;
        nav_msgs::Path _mpc_trackTraj;


        AD<double> cost_distx, cost_disty, cost_etheta;
        AD<double> cost_acc_x,  cost_acc_y, cost_angacc;

        string _file_debug_path;
        fstream file_debug;
};

#endif /* FG_evalue_H */