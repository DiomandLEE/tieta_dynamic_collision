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
#include "JointTrajPub/Angles.h"
#include "JointTrajPub/AnglesList.h"
typedef CPPAD_TESTVECTOR(double) Dvector;
using namespace std;
using CppAD::AD;

class FG_eval
{
    public:
        //构造函数
        FG_eval();
        FG_eval(JointTrajPub::AnglesList _trackTraj, Collision_Check &collisioncheck, vector<Eigen::Vector3d> tf_state);

        //加载public成员的参数
        void LoadParams(const std::map<string, double> &params);
        //todo这里给了行人在init时的位置，就可以用速度计算了
        AD<double> calculate_Obscost(const Dvector objA, const Dvector objB, const double d_threshold, const double weight);
        AD<double> calculate_selfcollision(const Dvector objA, const Dvector objB, const double d_threshold, const double weight);

        //定义目标函数F和约束信息函数G  MPC implementation (cost func & constraints)
        typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

        // fg: function that evaluates the objective and constraints using the syntax
        void operator()(ADvector &fg, const ADvector &vars);

        //Eigen::VectorXd coeffs;
        double _dt;
        double _obs_vel;
        double _w_distx, _w_disty, _w_etheta, _w_vel, _w_angvel,
                _w_jnt, _w_jntvel,
                _w_base_collision, _w_shoulder_collision, _w_elbow_collision, _w_wrist_collision, _w_gripper_collision;
        int _mpc_steps, _x_start, _y_start, _theta_start, _vx_start, _vy_start, _angvel_start;
        //UR5e
        int _joint1_start, _joint2_start, _joint3_start, _joint4_start, _joint5_start, _joint6_start,
                _jntvel1_start, _jntvel2_start, _jntvel3_start, _jntvel4_start, _jntvel5_start, _jntvel6_start;

        JointTrajPub::AnglesList _mpc_trackTraj;
        Collision_Check _collision_check;
        //碰撞安全阈值
        double _base_threshold, _shoulder_threshold, _elbow_threshold, _wrist_threshold, _gripper_threshold;
        //动态障碍物阈值
        double _pedestrian_threshold, _pedestrian_vel;
        vector<Eigen::Vector3d> _init_sphere;

        AD<double> cost_distx, cost_disty, cost_etheta, cost_jnt1, cost_jnt2, cost_jnt3, cost_jnt4, cost_jnt5, cost_jnt6;
        AD<double> cost_vx, cost_vy, cost_angvel, cost_jntvel1, cost_jntvel2, cost_jntvel3, cost_jntvel4, cost_jntvel5, cost_jntvel6;

        string _file_debug_path;
        fstream file_debug;
};

#endif /* FG_evalue_H */