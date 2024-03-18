#ifndef FG_evalue_H
#define FG_evalue_H

#include <NLsolver/cppad/ipopt/solve.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "ros/ros.h"

//tf include
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/message_filter.h>
#include <tf/transform_datatypes.h>

#include <fstream>
#include <iostream>
#include <string>
#include <iomanip>

//#include "tieta_mpc_sim_demo/Collision_Check.h"
#include "JointTrajPub/Angles.h"
#include "JointTrajPub/AnglesList.h"
typedef CPPAD_TESTVECTOR(double) Dvector;
//typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
using namespace std;
using CppAD::AD;

class FG_eval
{
    public:
        //构造函数
        FG_eval();
        FG_eval(JointTrajPub::AnglesList _trackTraj, vector<Eigen::Vector3d> tf_state, bool _flag, int _nums);

        //加载public成员的参数
        void LoadParams(const std::map<string, double> &params);
        //todo这里给了行人在init时的位置，就可以用速度计算了

        AD<double> calculate_terminalCost(tf::Vector3 ee_position, tf::Vector3 ee_orientation);

        //barried func
        AD<double> barried_func_arm_(AD<double> distance_);
        AD<double> barried_func_base_(AD<double> distance_);

        typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

        int casadi_shoulder_sphere(ADvector arg, ADvector &res);
        int casadi_elbow_sphere(ADvector arg, ADvector &res);
        int casadi_wrist_sphere(ADvector arg, ADvector &res);
        int casadi_gripper_sphere(ADvector arg, ADvector &res);
        int casadi_tool_pose(ADvector arg, ADvector &res);
        int casadi_base_lf(ADvector arg, ADvector &res);
        int casadi_base_rf(ADvector arg, ADvector &res);
        int casadi_base_lr(ADvector arg, ADvector &res);
        int casadi_base_rr(ADvector arg, ADvector &res);

        //定义目标函数F和约束信息函数G  MPC implementation (cost func & constraints)


        // fg: function that evaluates the objective and constraints using the syntax
        void operator()(ADvector &fg, const ADvector &vars);

        //for termianl EE constraint
        vector<double> _w_vector_terminal;

        //Eigen::VectorXd coeffs;
        double _dt;
        double _obs_vel;
        double _w_distx, _w_disty, _w_etheta, _w_vel, _w_angvel, _w_acc, _w_angacc,
                _w_jnt, _w_jnt2, _w_jntvel, _w_jntacc,
                _w_base_collision, _w_shoulder_collision, _w_elbow_collision, _w_wrist_collision, _w_gripper_collision;
        //用来改变 机械臂的sphere和行人的圆柱 的距离惩罚函数的SIGMOD系数
        double _barried_func_arm_w, _barried_func_arm_r, _barried_func_arm_m, _barried_func_arm_n;
        //用来改变 base的sphere和行人的圆柱 的距离惩罚函数的SIGMOD系数
        double _barried_func_base_w, _barried_func_base_r, _barried_func_base_m, _barried_func_base_n;

        int _mpc_steps, _x_start, _y_start, _theta_start, _vx_start, _vy_start, _angvel_start;
        //UR5e
        int _joint1_start, _joint2_start, _joint3_start, _joint4_start, _joint5_start, _joint6_start,
                _jntvel1_start, _jntvel2_start, _jntvel3_start, _jntvel4_start, _jntvel5_start, _jntvel6_start;

        JointTrajPub::AnglesList _mpc_trackTraj;

        //碰撞安全阈值
        double _base_threshold, _shoulder_threshold, _elbow_threshold, _wrist_threshold, _gripper_threshold;
        //动态障碍物阈值
        double _pedestrian_threshold, _pedestrian_vel_x, _pedestrian_vel_y;
        vector<Eigen::Vector3d> _init_sphere;

        //terminal flag
        bool _terminal_flag;
        int _terminal_nums;
        //terminal hard constraint
        double EE_X, EE_Y, EE_Z, EE_ROLL, EE_PITCH, EE_YAW;
        double _w_hard_EE_tool;

        AD<double> cost_distx, cost_disty, cost_etheta, cost_jnt1, cost_jnt2, cost_jnt3, cost_jnt4, cost_jnt5, cost_jnt6;
        AD<double> cost_vx, cost_vy, cost_angvel, cost_jntvel1, cost_jntvel2, cost_jntvel3, cost_jntvel4, cost_jntvel5, cost_jntvel6;

        string _file_debug_path;
        fstream file_debug;
};

#endif /* FG_evalue_H */