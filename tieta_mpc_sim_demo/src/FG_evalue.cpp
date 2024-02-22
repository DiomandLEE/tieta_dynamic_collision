#include "tieta_mpc_sim_demo/FG_evalue.h"
//#include <cppad/cppad.hpp>
#include <NLsolver/cppad/ipopt/solve.hpp>
#include <Eigen/Core>
typedef CPPAD_TESTVECTOR(double) Dvector;
using CppAD::AD;
//! 认为Collision Check是在这里应用的

// =========================================
// FG_eval class definition implementation.
// =========================================
FG_eval::FG_eval()
{}
//构造函数---赋初值
FG_eval::FG_eval(JointTrajPub::AnglesList _trackTraj, Collision_Check &collisioncheck, vector<Eigen::Vector3d> tf_state)
    : _mpc_trackTraj(_trackTraj), _collision_check(collisioncheck), _init_sphere(tf_state)
{
    //采样时间和控制率频率
    _dt = 0.1; //sec
    _w_distx     = 100;
    _w_disty     = 100;
    _w_etheta    = 100;
    _w_vel     = 1;
    _w_angvel   = 100;
    _w_jnt = 100;
    _w_jntvel = 100;
    _w_base_collision = 500;
    _w_shoulder_collision = 500;
    _w_elbow_collision = 500;
    _w_wrist_collision = 500;
    _w_gripper_collision = 500;

    _mpc_steps   = 20;
    //state index
    //9个位置变量，(9 * mpc_step) * 1
    _x_start = 0;
    _y_start = _x_start + _mpc_steps;
    _theta_start = _y_start + _mpc_steps;
    _joint1_start = _theta_start + _mpc_steps;
    _joint2_start = _joint1_start + _mpc_steps;
    _joint3_start = _joint2_start + _mpc_steps;
    _joint4_start = _joint3_start + _mpc_steps;
    _joint5_start = _joint4_start + _mpc_steps;
    _joint6_start = _joint5_start + _mpc_steps;

    //9个速度变量，(9 * (mpc_step - 1)) * 1
    _vx_start = _joint6_start + _mpc_steps;
    _vy_start = _vx_start + _mpc_steps - 1;
    _angvel_start = _vy_start + _mpc_steps - 1;
    _jntvel1_start = _angvel_start + _mpc_steps;
    _jntvel2_start = _jntvel1_start + _mpc_steps - 1;
    _jntvel3_start = _jntvel2_start + _mpc_steps - 1;
    _jntvel4_start = _jntvel3_start + _mpc_steps - 1;
    _jntvel5_start = _jntvel4_start + _mpc_steps - 1;
    _jntvel6_start = _jntvel5_start + _mpc_steps - 1;

    //? 来看一下，计算误差用到的position
    cost_distx = 0;
    cost_disty = 0;
    cost_etheta = 0;
    cost_jnt1 = 0;
    cost_jnt2 = 0;
    cost_jnt3 = 0;
    cost_jnt4 = 0;
    cost_jnt5 = 0;
    cost_jnt6 = 0;

    cost_vx = 0;
    cost_vy = 0;
    cost_angvel = 0;
    cost_jntvel1 = 0;
    cost_jntvel2 = 0;
    cost_jntvel3 = 0;
    cost_jntvel4 = 0;
    cost_jntvel5 = 0;
    cost_jntvel6 = 0;

    _file_debug_path = " ";
}

void FG_eval::LoadParams(const std::map<string, double> &params)
{
    _dt = params.find("DT") != params.end() ? params.at("DT") : _dt;
    _mpc_steps = params.find("STEPS") != params.end()    ? params.at("STEPS") : _mpc_steps;
    _w_distx   = params.find("W_DISTX") != params.end()   ? params.at("W_DISTX") : _w_distx;
    _w_disty = params.find("W_DISTY") != params.end() ? params.at("W_DISTY") : _w_disty;
    _w_etheta = params.find("W_EPSI") != params.end() ? params.at("W_EPSI") : _w_etheta;

    _w_vel   = params.find("W_VEL") != params.end()     ? params.at("W_VEL") : _w_vel;
    _w_angvel = params.find("W_ANGVEL") != params.end() ? params.at("W_ANGVEL") : _w_angvel;

    _w_jnt = params.find("W_JOINT") != params.end()     ? params.at("W_JOINT") : _w_jnt;
    _w_jntvel = params.find("W_JNTVEL") != params.end() ? params.at("W_JNTVEL") : _w_jntvel;

    _w_base_collision = params.find("COLLISION_BASE_WEIGHT") != params.end() ? params.at("COLLISION_BASE_WEIGHT") : _w_base_collision;
    _w_shoulder_collision = params.find("COLLISION_SHOULDER_WEIGHT") != params.end() ? params.at("COLLISION_SHOULDER_WEIGHT") : _w_shoulder_collision;
    _w_elbow_collision = params.find("COLLISION_ELBOW_WEIGHT") != params.end() ? params.at("COLLISION_ELBOW_WEIGHT") : _w_elbow_collision;
    _w_wrist_collision = params.find("COLLISION_WRIST_WEIGHT") != params.end() ? params.at("COLLISION_WRIST_WEIGHT") : _w_wrist_collision;
    _w_gripper_collision = params.find("COLLISION_GRIPPER_WEIGHT") != params.end() ? params.at("COLLISION_GRIPPER_WEIGHT") : _w_gripper_collision;

    _base_threshold = params.find("BASE_THRESHOLD") != params.end() ? params.at("BASE_THRESHOLD") : _base_threshold;
    _shoulder_threshold = params.find("SHOULDER_THRESHOLD") != params.end() ? params.at("SHOULDER_THRESHOLD") : _shoulder_threshold;
    _elbow_threshold = params.find("ELBOW_THRESHOLD") != params.end() ? params.at("ELBOW_THRESHOLD") : _elbow_threshold;
    _wrist_threshold = params.find("WRIST_THRESHOLD") != params.end() ? params.at("WRIST_THRESHOLD") : _wrist_threshold;
    _gripper_threshold = params.find("GRIPPER_THRESHOLD") != params.end() ? params.at("GRIPPER_THRESHOLD") : _gripper_threshold;
    _pedestrian_threshold = params.find("PEDESTRIAN_THRESHOLD") != params.end() ? params.at("PEDESTRIAN_THRESHOLD") : _pedestrian_threshold;
    _pedestrian_vel = params.find("PEDESTRIAN_VELOCITY") != params.end() ? params.at("PEDESTRIAN_VELOCITY") : _pedestrian_vel;

    //state index
    //9个位置变量，(9 * mpc_step) * 1
    _x_start = 0;
    _y_start = _x_start + _mpc_steps;
    _theta_start = _y_start + _mpc_steps;
    _joint1_start = _theta_start + _mpc_steps;
    _joint2_start = _joint1_start + _mpc_steps;
    _joint3_start = _joint2_start + _mpc_steps;
    _joint4_start = _joint3_start + _mpc_steps;
    _joint5_start = _joint4_start + _mpc_steps;
    _joint6_start = _joint5_start + _mpc_steps;

    //9个速度变量，(9 * (mpc_step - 1)) * 1
    _vx_start = _joint6_start + _mpc_steps;
    _vy_start = _vx_start + _mpc_steps - 1;
    _angvel_start = _vy_start + _mpc_steps - 1;
    _jntvel1_start = _angvel_start + _mpc_steps;
    _jntvel2_start = _jntvel1_start + _mpc_steps - 1;
    _jntvel3_start = _jntvel2_start + _mpc_steps - 1;
    _jntvel4_start = _jntvel3_start + _mpc_steps - 1;
    _jntvel5_start = _jntvel4_start + _mpc_steps - 1;
    _jntvel6_start = _jntvel5_start + _mpc_steps - 1;
}

AD<double> FG_eval::calculate_Obscost(const Dvector objA, const Dvector objB, const double d_threshold, const double weight) {
    // 计算距离的平方 d^2
    //! 这个是圆柱体和球体之间的距离，所以只要在一个平面上就好
    AD<double> d_squared = 0.0;
    for (int i = 0; i < 2; ++i) {
        d_squared += CppAD::pow(objA[i] - objB[i], 2);
    }
    AD<double> distance_cost = 0.0;

    // 若 d^2 大于 d_threshold^2，则输出 (d - d_threshold)^2
    if (d_squared > CppAD::pow(d_threshold, 2)) {
        distance_cost = weight * CppAD::pow((CppAD::sqrt(d_squared) - d_threshold), 2);
    }

    return distance_cost; // 否则输出距离差的平方
}

AD<double> FG_eval::calculate_selfcollision(const Dvector objA, const Dvector objB, const double d_threshold, const double weight){
    // 计算距离的平方 d^2
    //!这个是要计算三维的，因为是自碰撞，两个球之间的距离
    AD<double> d_squared = 0.0;
    for (int i = 0; i < 3; ++i) {
        d_squared += CppAD::pow(objA[i] - objB[i], 2);
    }
    AD<double> distance_cost = 0.0;

    // 若 d^2 大于 d_threshold^2，则输出 (d - d_threshold)^2
    if (d_squared > CppAD::pow(d_threshold, 2)) {
        distance_cost = weight * CppAD::pow((CppAD::sqrt(d_squared) - d_threshold), 2);
    }

    return distance_cost; // 否则输出距离差的平方
}


void FG_eval::operator()(ADvector& fg, const ADvector& vars)
{
    //first,fg[0] represent for Objective Function
    fg[0] = 0;
    cost_distx = 0;
    cost_disty = 0;
    cost_etheta = 0;
    cost_jnt1 = 0;
    cost_jnt2 = 0;
    cost_jnt3 = 0;
    cost_jnt4 = 0;
    cost_jnt5 = 0;
    cost_jnt6 = 0;

    cost_vx = 0;
    cost_vy = 0;
    cost_angvel = 0;
    cost_jntvel1 = 0;
    cost_jntvel2 = 0;
    cost_jntvel3 = 0;
    cost_jntvel4 = 0;
    cost_jntvel5 = 0;
    cost_jntvel6 = 0;
    //! 初步感觉是_w_disty没有给的原因，而且角度项的cost很大



    for (int i = 0; i < _mpc_steps; i++)
    {
        std::vector<double> current_joint_position(9);
        current_joint_position = {CppAD::Value(CppAD::Var2Par(vars[_x_start + i])), CppAD::Value(CppAD::Var2Par(vars[_y_start + i])), CppAD::Value(CppAD::Var2Par(vars[_theta_start + i])),
            CppAD::Value(CppAD::Var2Par(vars[_joint1_start + i])), CppAD::Value(CppAD::Var2Par(vars[_joint2_start + i])), CppAD::Value(CppAD::Var2Par(vars[_joint3_start + i])),
            CppAD::Value(CppAD::Var2Par(vars[_joint4_start + i])), CppAD::Value(CppAD::Var2Par(vars[_joint5_start + i])), CppAD::Value(CppAD::Var2Par(vars[_joint6_start + i]))};
        //CppAD::Value(CppAD::Var2Par(vars[_x_start + i]));
        //!计算当前的各个sphere的位置，std::vector<Dvector>
        _collision_check.calculate_sphere_position(current_joint_position);
        std::vector<Dvector> current_sphere_position = _collision_check.get_sphere_position();
        // todo z represent theta for traj_pub
        //!这不和正常的double 也可以做减法么

        //debug 这是位置误差
        fg[0] += _w_distx * CppAD::pow(vars[_x_start + i] - _mpc_trackTraj.AnglesList[i].base_x, 2); // x error
        fg[0] += _w_disty * CppAD::pow(vars[_y_start + i] - _mpc_trackTraj.AnglesList[i].base_y, 2); // y error
        fg[0] += _w_etheta * CppAD::pow(vars[_theta_start + i] - _mpc_trackTraj.AnglesList[i].base_theta, 2); // theta error
        fg[0] += _w_jnt * CppAD::pow(vars[_joint1_start + i] - _mpc_trackTraj.AnglesList[i].joint1, 2); // joint1 error
        fg[0] += _w_jnt * CppAD::pow(vars[_joint2_start + i] - _mpc_trackTraj.AnglesList[i].joint2, 2); // joint2 error
        fg[0] += _w_jnt * CppAD::pow(vars[_joint3_start + i] - _mpc_trackTraj.AnglesList[i].joint3, 2); // joint3 error
        fg[0] += _w_jnt * CppAD::pow(vars[_joint4_start + i] - _mpc_trackTraj.AnglesList[i].joint4, 2); // joint4 error
        fg[0] += _w_jnt * CppAD::pow(vars[_joint5_start + i] - _mpc_trackTraj.AnglesList[i].joint5, 2); // joint5 error
        fg[0] += _w_jnt * CppAD::pow(vars[_joint6_start + i] - _mpc_trackTraj.AnglesList[i].joint6, 2); // joint6 error

        //debug 计算障碍物误差
        Dvector pedestrian_predpos(3);
        pedestrian_predpos[0] = _init_sphere[0][0] + i * _dt * _pedestrian_vel;
        pedestrian_predpos[1] = _init_sphere[0][1] + i * _dt * _pedestrian_vel;
        pedestrian_predpos[2] = _init_sphere[0][2];
        //base
        fg[0] += calculate_Obscost(pedestrian_predpos, current_sphere_position[0], _pedestrian_threshold + _base_threshold, _w_base_collision);
        fg[0] += calculate_Obscost(pedestrian_predpos, current_sphere_position[1], _pedestrian_threshold + _base_threshold, _w_base_collision);
        fg[0] += calculate_Obscost(pedestrian_predpos, current_sphere_position[2], _pedestrian_threshold + _base_threshold, _w_base_collision);
        fg[0] += calculate_Obscost(pedestrian_predpos, current_sphere_position[3], _pedestrian_threshold + _base_threshold, _w_base_collision);
        //shoulder
        fg[0] += calculate_Obscost(pedestrian_predpos, current_sphere_position[4], _pedestrian_threshold + _shoulder_threshold, _w_shoulder_collision);
        //elbow
        fg[0] += calculate_Obscost(pedestrian_predpos, current_sphere_position[5], _pedestrian_threshold + _elbow_threshold, _w_elbow_collision);
        //wrist
        fg[0] += calculate_Obscost(pedestrian_predpos, current_sphere_position[6], _pedestrian_threshold + _wrist_threshold, _w_wrist_collision);
        //gripper
        fg[0] += calculate_Obscost(pedestrian_predpos, current_sphere_position[7], _pedestrian_threshold + _gripper_threshold, _w_gripper_collision);
        //!self collision 因为不考虑速度先，所以用w_vel代替，自碰撞的系数
        fg[0] += calculate_selfcollision(current_sphere_position[4], current_sphere_position[6], _shoulder_threshold + _elbow_threshold + _gripper_threshold, _w_vel);
        // 计算cost
        // 需要改一下parseCSV的traj代码，将theta放在position.z里
        // cost_distx += _w_distx * CppAD::pow(vars[_x_start + i] - _mpc_trackTraj.poses[i].pose.position.x, 2);
        // cost_disty +=  (_w_disty * CppAD::pow(vars[_y_start + i] - _mpc_trackTraj.poses[i].pose.position.y, 2));
        // cost_etheta +=  (_w_etheta * CppAD::pow(vars[_theta_start + i] - _mpc_trackTraj.poses[i].pose.position.z, 2));
    }
    cout << "-----------------------------------------------" <<endl;

    //!Minimize the use of actuators.加速度a和角加速度w
    //diff -> get the angle-acc & jerk
    // for (int i = 0; i < _mpc_steps - 2; i++) {
    //     fg[0] += _w_angacc * CppAD::pow(vars[_angvel_start + i + 1] - vars[_angvel_start + i], 2);
    //     fg[0] += _w_acc * CppAD::pow(vars[_vx_start + i + 1] - vars[_vx_start + i], 2);
    //     fg[0] += _w_acc * CppAD::pow(vars[_vy_start + i + 1] - vars[_vy_start + i], 2);

    //     cost_acc_x += _w_acc * CppAD::pow(vars[_vx_start + i + 1] - vars[_vx_start + i], 2);
    //     cout << "真的家的??? ACC_X :   " << cost_acc_x << endl;
    //     cost_acc_y += _w_acc * CppAD::pow(vars[_vy_start + i + 1] - vars[_vy_start + i], 2);
    //     cout << "真的家的??? ACC_Y :   " << cost_acc_y << endl;
    //     cost_angacc += _w_angacc * CppAD::pow(vars[_angvel_start + i + 1] - vars[_angvel_start + i], 2);
    //     cout << "真的家的??? ACC_THETA :   " << cost_angacc << endl;
    // }

    // fg[x] for constraints
    //! 初始的约束：其实就是把自变量的值付过来
    // Initial constraints
    // 等式约束，为了规定初值
    //! 目标函数F，约束函数G

    fg[1 + _x_start] = vars[_x_start];
    fg[1 + _y_start] = vars[_y_start];
    fg[1 + _theta_start] = vars[_theta_start];
    //! 速度初值肯定不能给啊，本来就是算这个的，v[0]
    fg[1 + _joint1_start] = vars[_joint1_start];
    fg[1 + _joint2_start] = vars[_joint2_start];
    fg[1 + _joint3_start] = vars[_joint3_start];
    fg[1 + _joint4_start] = vars[_joint4_start];
    fg[1 + _joint5_start] = vars[_joint5_start];
    fg[1 + _joint6_start] = vars[_joint6_start];


    // cout << "vars.size() " << vars.size() << endl;
    // cout << "fg.size() " << fg.size() << endl;
    // cout << "fg最后一项的index:" << 2 + _theta_start + _mpc_steps -2 << endl;
    //debug fg.size() 等于 ng+1 , ng是g(x)的数量,约束一共又两部分组成,纯x的不等式约束,和g(x)的等式&不等式约束

    //系统状态方程->视为等式约束
    for (int i = 0; i < _mpc_steps - 1; i++)
    {
        // The state at time t+1 .
        AD<double> x1 = vars[_x_start + i + 1];
        AD<double> y1 = vars[_y_start + i + 1];
        AD<double> theta1 = vars[_theta_start + i + 1];
        // The state at time t+1 .
        AD<double> joint1_angle_t1 = vars[_joint1_start + i + 1];
        AD<double> joint2_angle_t1 = vars[_joint2_start + i + 1];
        AD<double> joint3_angle_t1 = vars[_joint3_start + i + 1];
        AD<double> joint4_angle_t1 = vars[_joint4_start + i + 1];
        AD<double> joint5_angle_t1 = vars[_joint5_start + i + 1];
        AD<double> joint6_angle_t1 = vars[_joint6_start + i + 1];

        // The state at time t.
        AD<double> x0 = vars[_x_start + i];
        AD<double> y0 = vars[_y_start + i];
        AD<double> theta0 = vars[_theta_start + i];
        // The state at time t.
        AD<double> joint1_angle_t0 = vars[_joint1_start + i];
        AD<double> joint2_angle_t0 = vars[_joint2_start + i];
        AD<double> joint3_angle_t0 = vars[_joint3_start + i];
        AD<double> joint4_angle_t0 = vars[_joint4_start + i];
        AD<double> joint5_angle_t0 = vars[_joint5_start + i];
        AD<double> joint6_angle_t0 = vars[_joint6_start + i];

        // Only consider the actuation at time t.仅考虑时间t时的输入（驱动）。
        AD<double> vx_ = vars[_vx_start + i];
        AD<double> vy_ = vars[_vy_start + i];
        AD<double> w_ = vars[_angvel_start + i];
        // Only consider the actuation at time t.仅考虑时间t时的输入（驱动）。
        AD<double> joint1_speed_ = vars[_jntvel1_start + i];
        AD<double> joint2_speed_ = vars[_jntvel2_start + i];
        AD<double> joint3_speed_ = vars[_jntvel3_start + i];
        AD<double> joint4_speed_ = vars[_jntvel4_start + i];
        AD<double> joint5_speed_ = vars[_jntvel5_start + i];
        AD<double> joint6_speed_ = vars[_jntvel6_start + i];

        //! 使用运动学的积分方程,这些都是线性的，所以不需要什么运动学方程，直接乘时间就行。毕竟都是在world系下进行的
        // TODO: Setup the rest of the model constraints在这里将系统的状态方程填入
        fg[2 + _x_start + i] = x1 - (x0 + vx_ * _dt);
        fg[2 + _y_start + i] = y1 - (y0 + vy_ * _dt);
        fg[2 + _theta_start + i] = theta1 - (theta0 + w_ * _dt);

        fg[2 + _joint1_start + i] = joint1_angle_t1 - (joint1_angle_t0 + joint1_speed_ * _dt);
        fg[2 + _joint2_start + i] = joint2_angle_t1 - (joint2_angle_t0 + joint2_speed_ * _dt);
        fg[2 + _joint3_start + i] = joint3_angle_t1 - (joint3_angle_t0 + joint3_speed_ * _dt);
        fg[2 + _joint4_start + i] = joint4_angle_t1 - (joint4_angle_t0 + joint4_speed_ * _dt);
        fg[2 + _joint5_start + i] = joint5_angle_t1 - (joint5_angle_t0 + joint5_speed_ * _dt);
        fg[2 + _joint6_start + i] = joint6_angle_t1 - (joint6_angle_t0 + joint6_speed_ * _dt);
    }
}