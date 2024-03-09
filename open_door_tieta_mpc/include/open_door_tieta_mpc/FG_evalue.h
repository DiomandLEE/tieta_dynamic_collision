#ifndef FG_evalue_H
#define FG_evalue_H

//#include <NLsolver/cppad/ipopt/solve.hpp>
#include <cppad/ipopt/solve.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "ros/ros.h"
#include <Eigen/Geometry>

//tf include
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/message_filter.h>
#include <tf/transform_datatypes.h>

#include <fstream>
#include <iostream>
#include <string>
#include <iomanip>

#include "DoorTrajPub/Angles.h"
#include "DoorTrajPub/AnglesList.h"
typedef CPPAD_TESTVECTOR(double) Dvector;
//typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
using namespace std;
using CppAD::AD;

class FG_eval
{
    public:
        //构造函数
        FG_eval();
        FG_eval(DoorTrajPub::AnglesList _trackTraj, vector<Eigen::Vector3d> tf_state, bool _flag, int _nums);

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
                _w_base_collision, _w_shoulder_collision, _w_elbow_collision, _w_wrist_collision, _w_gripper_collision;
        //用来改变 机械臂的sphere和行人的圆柱 的距离惩罚函数的SIGMOD系数
        double _barried_func_arm_w, _barried_func_arm_r, _barried_func_arm_m, _barried_func_arm_n;
        //用来改变 base的sphere和行人的圆柱 的距离惩罚函数的SIGMOD系数
        double _barried_func_base_w, _barried_func_base_r, _barried_func_base_m, _barried_func_base_n;

        int _mpc_steps, _x_start, _y_start, _theta_start, _vx_start, _vy_start, _angvel_start;

        DoorTrajPub::AnglesList _mpc_trackTraj;

        //碰撞安全阈值
        double _base_threshold, _shoulder_threshold, _elbow_threshold, _wrist_threshold, _gripper_threshold;
        //动态障碍物阈值
        double _pedestrian_threshold, _pedestrian_vel;
        vector<Eigen::Vector3d> _init_sphere;

        Eigen::Vector3d tv_normal, closet_right_normal, closet_front_normal;
        Eigen::Vector3d tv_point, closet_point; //closet point for 柜子右面和前面，计算距离用的
        //上面这个两个point是为了计算法向量

        Eigen::Vector3d closet_right_proj_, closet_front_proj_; //这个是求投影的vector
        Eigen::Vector3d closet_right_proj_start_point, closet_front_proj_start_point; //这个是求投影的起始点
        //对应柜子的右后方的点 对应柜子的左前点即closet_bottom_left_door_link x
        Eigen::Vector4d door_init_normal; //无论门在什么角度，门系上的法向量都是它
        Eigen::Vector4d door_bottom_tip_point; //in door frame
        Eigen::Vector4d door_handle_point; //in door
        Eigen::Vector3d door_link_origin; //in world

        //旋转轴
        Eigen::Vector3d door_axis;

        //terminal flag //todo 这个应该是用不着了，因为没有next step
        bool _terminal_flag;
        int _terminal_nums;

        AD<double> cost_distx, cost_disty, cost_etheta;
        AD<double> cost_vx, cost_vy, cost_angvel;

        string _file_debug_path;
        fstream file_debug;
};

#endif /* FG_evalue_H */