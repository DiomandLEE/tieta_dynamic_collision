#include "mpc_rate/FG_evalue.h"
//#include <cppad/cppad.hpp>
#include <NLsolver/cppad/ipopt/solve.hpp>
#include <Eigen/Core>

using CppAD::AD;

// =========================================
// FG_eval class definition implementation.
// =========================================

//构造函数---赋初值
FG_eval::FG_eval(nav_msgs::Path _trackTraj)
{
    _mpc_trackTraj = _trackTraj;
    //采样时间和控制率频率
    _dt = 0.01; //sec
    _w_distx     = 100;
    //_w_disty     = 100;
    _w_etheta    = 100;
    _w_vel     = 1;
    _w_angvel   = 100;
    _w_acc   = 50;
    _w_angacc = 0;

    _mpc_steps   = 40;
    //state index
    _x_start     = 0;
    _y_start     = _x_start + _mpc_steps;
    _theta_start   = _y_start + _mpc_steps;
    //input index
    _vx_start     = _theta_start + _mpc_steps;
    _vy_start = _vx_start + _mpc_steps - 1;
    _angvel_start = _vy_start + _mpc_steps - 1; // 控制量少一个timestep
    //cout << "构造函数——angvel_start" << _angvel_start << endl;
    //cout << "构造函数——distx_start" << _distx_start << endl;
    //? 来看一下，计算误差用到的position
    /*
    for(int i = 0; i < _mpc_trackTraj.poses.size(); i++)
    {
        cout << "第" << i+1 << "个轨迹点" << _mpc_trackTraj.poses[i].pose.position.x << endl;
    }
    */
    cost_distx = 0;
    cost_disty = 0;
    cost_etheta = 0;

    cost_acc_x = 0.0;
    cost_acc_y = 0.0;
    cost_angacc = 0.0;

    _file_debug_path = " ";
}

void FG_eval::LoadParams(const std::map<string, double> &params)
{
    _dt = params.find("DT") != params.end() ? params.at("DT") : _dt;
    _mpc_steps = params.find("STEPS") != params.end()    ? params.at("STEPS") : _mpc_steps;
    //_ref_distx   = params.find("REF_DISTX") != params.end()  ? params.at("REF_DISTX") : _ref_distx;
    //_ref_etheta  = params.find("REF_ETHETA") != params.end() ? params.at("REF_ETHETA") : _ref_etheta;
    //_ref_vel   = params.find("REF_V") != params.end()    ? params.at("REF_V") : _ref_vel;

    _w_distx   = params.find("W_DISTX") != params.end()   ? params.at("W_DISTX") : _w_distx;
    _w_disty = params.find("W_DISTY") != params.end() ? params.at("W_DISTY") : _w_disty;
    _w_etheta = params.find("W_EPSI") != params.end() ? params.at("W_EPSI") : _w_etheta;
    _w_vel   = params.find("W_V") != params.end()     ? params.at("W_V") : _w_vel;
    _w_angvel = params.find("W_ANGVEL") != params.end() ? params.at("W_ANGVEL") : _w_angvel;
    _w_acc = params.find("W_ACC") != params.end()     ? params.at("W_ACC") : _w_acc;
    _w_angacc = params.find("W_DANGVEL") != params.end() ? params.at("W_DANGVEL") : _w_angacc;
    //_w_jerk = params.find("W_JERK") != params.end()     ? params.at("W_JERK") : _w_jerk;

    _x_start     = 0;
    _y_start     = _x_start + _mpc_steps;
    _theta_start   = _y_start + _mpc_steps;
    _vx_start     = _theta_start + _mpc_steps;
    //! 这个是用在g(x)中的等式约束，去表达系统的状态方程的
    _vy_start = _vx_start + _mpc_steps - 1;
    _angvel_start = _vy_start + _mpc_steps - 1;

    // file_debug.open(_file_debug_path);
    // if(file_debug.is_open())
    //     ROS_INFO("Class FG_eval : debug_csv/csv file has been open !");
    // else
    //     ROS_ERROR("Class FG_eval : Cannot open debug file: %s", _file_debug_path.c_str());

    // cout << _x_start << "," <<  _y_start << "," <<  _theta_start << endl;
    // cout << _vx_start << "," << _vy_start << "," << _angvel_start << endl;
}

void FG_eval::operator()(ADvector& fg, const ADvector& vars)
{
    //first,fg[0] represent for Objective Function
    fg[0] = 0;
    cost_distx = 0;
    cost_disty = 0;
    cost_etheta = 0;

    cost_acc_x = 0.0;
    cost_acc_y = 0.0;
    cost_angacc = 0.0;

    cout << "@@@@@@@@ 获得传进来的track 轨迹 segment @@@@@@@@" << endl;
    cout << "---- 权重系数： "
         << "_w_distx: " << _w_distx << " , "
         << "_w_disty: " << _w_disty << " , "
         << "_w_etheta: " << _w_etheta << " , "
         << "_w_acc: " << _w_acc << " , "
         << "_w_angacc: " << _w_angacc << " , " << endl;

    file_debug << "@@@@@@@@ 获得传进来的track 轨迹 segment @@@@@@@@" << endl;
    file_debug << "---- 权重系数： "
         << "_w_distx: " << _w_distx << " , "
         << "_w_disty: " << _w_disty << " , "
         << "_w_etheta: " << _w_etheta << " , "
         << "_w_acc: " << _w_acc << " , "
         << "_w_angacc: " << _w_angacc << " , " << endl;
    //! 初步感觉是_w_disty没有给的原因，而且角度项的cost很大

    for (int i = 0; i < _mpc_steps; i++)
    {
        // for debug

        cout << "--- 第" << i << "个点: " << "x: " << _mpc_trackTraj.poses[i].pose.position.x << " , "
            << "y: " << _mpc_trackTraj.poses[i].pose.position.y << " , "
            << "theta: " << _mpc_trackTraj.poses[i].pose.position.z << endl;

        file_debug << "--- 第" << i << "个点: " << "x: " << _mpc_trackTraj.poses[i].pose.position.x << " , "
            << "y: " << _mpc_trackTraj.poses[i].pose.position.y << " , "
            << "theta: " << _mpc_trackTraj.poses[i].pose.position.z << endl;

        // todo z represent theta for traj_pub
        fg[0] += _w_distx * CppAD::pow(vars[_x_start + i] - _mpc_trackTraj.poses[i].pose.position.x, 2); // cross deviation error
        fg[0] += _w_disty * CppAD::pow(vars[_y_start + i] - _mpc_trackTraj.poses[i].pose.position.y, 2); // heading error
        fg[0] += _w_etheta * CppAD::pow(vars[_theta_start + i] - _mpc_trackTraj.poses[i].pose.position.z, 2); // speed error
        // 计算cost
        //需要改一下parseCSV的traj代码，将theta放在position.z里
        cost_distx +=  _w_distx * CppAD::pow(vars[_x_start + i] - _mpc_trackTraj.poses[i].pose.position.x, 2);
        cost_disty +=  (_w_disty * CppAD::pow(vars[_y_start + i] - _mpc_trackTraj.poses[i].pose.position.y, 2));
        cost_etheta +=  (_w_etheta * CppAD::pow(vars[_theta_start + i] - _mpc_trackTraj.poses[i].pose.position.z, 2));
    }
    cout << "-----------------------------------------------" <<endl;
    //cout << "x direction distance error: " << cost_distx << endl;
    //cout << "_angvel_start: " << _angvel_start << endl;

    //Minimize the use of actuators.加速度a和角加速度w
    //diff -> get the angle-acc & jerk
    for (int i = 0; i < _mpc_steps - 2; i++) {
        fg[0] += _w_angacc * CppAD::pow(vars[_angvel_start + i + 1] - vars[_angvel_start + i], 2);
        fg[0] += _w_acc * CppAD::pow(vars[_vx_start + i + 1] - vars[_vx_start + i], 2);
        fg[0] += _w_acc * CppAD::pow(vars[_vy_start + i + 1] - vars[_vy_start + i], 2);

        cost_acc_x += _w_acc * CppAD::pow(vars[_vx_start + i + 1] - vars[_vx_start + i], 2);
        cout << "真的家的??? ACC_X :   " << cost_acc_x << endl;
        cost_acc_y += _w_acc * CppAD::pow(vars[_vy_start + i + 1] - vars[_vy_start + i], 2);
        cout << "真的家的??? ACC_Y :   " << cost_acc_y << endl;
        cost_angacc += _w_angacc * CppAD::pow(vars[_angvel_start + i + 1] - vars[_angvel_start + i], 2);
        cout << "真的家的??? ACC_THETA :   " << cost_angacc << endl;
    }
    //cout << "DEBUG2" << endl;
    // fg[x] for constraints
    //! 初始的约束：其实就是把自变量的值付过来
    // Initial constraints
    // 等式约束，为了规定初值

    // cout << _x_start << "," <<  _y_start << "," <<  _theta_start << endl;
    // cout << _vx_start << "," << _vy_start << "," << _angvel_start << endl;
    fg[1 + _x_start] = vars[_x_start];
    //cout << "DEBUG2.1" << endl;
    fg[1 + _y_start] = vars[_y_start];
    //cout << "DEBUG2.2" << endl;
    fg[1 + _theta_start] = vars[_theta_start];
    //cout << "DEBUG2.3" << endl;

    // cout << "vars.size() " << vars.size() << endl;
    // cout << "fg.size() " << fg.size() << endl;
    // cout << "fg最后一项的index:" << 2 + _theta_start + _mpc_steps -2 << endl;
    //debug fg.size() 等于 ng+1 , ng是g(x)的数量,约束一共又两部分组成,纯x的不等式约束,和g(x)的等式&不等式约束

    // fg[1 + _vx_start] = vars[_vx_start];
    // cout << "DEBUG2.4" << endl;
    // fg[1 + _vy_start] = vars[_vy_start];
    // cout << "DEBUG2.5" << endl;
    // fg[1 + _angvel_start] = vars[_angvel_start];
    // cout << "DEBUG3" << endl;

    //系统状态方程->视为等式约束
    for (int i = 0; i < _mpc_steps - 1; i++)
    {
        // The state at time t+1 .
        AD<double> x1 = vars[_x_start + i + 1];
        AD<double> y1 = vars[_y_start + i + 1];
        AD<double> theta1 = vars[_theta_start + i + 1];

        // The state at time t.
        AD<double> x0 = vars[_x_start + i];
        AD<double> y0 = vars[_y_start + i];
        AD<double> theta0 = vars[_theta_start + i];

        // Only consider the actuation at time t.仅考虑时间t时的输入（驱动）。
        AD<double> vx_ = vars[_vx_start + i];
        AD<double> vy_ = vars[_vy_start + i];
        AD<double> w_ = vars[_angvel_start + i];
        //cout << "DEBUG4" << endl;

        //将速度转换到世界坐标系
        // AD<double> vx_map = CppAD::cos(theta0) * vx_ - CppAD::sin(theta0) * vy_;
        // AD<double> vy_map = CppAD::sin(theta0) * vx_ + CppAD::cos(theta0) * vy_;
        // AD<double> w_map = w_ + 0.0001;

        //! 使用运动学的积分方程
        AD<double> vx_map = (CppAD::sin(theta1) - CppAD::sin(theta0)) * vx_ + (CppAD::cos(theta1) - CppAD::cos(theta0)) * vy_;
        AD<double> vy_map = (-CppAD::cos(theta1) + CppAD::cos(theta1)) * vx_ + (CppAD::sin(theta1) - CppAD::sin(theta0)) * vy_;
        AD<double> w_map = w_ + 0.0001;
        // cout << "DEBUG5" << endl;

        // TODO: Setup the rest of the model constraints在这里将系统的状态方程填入
        //! 之后需要有坐标转换，在这不用是因为，只考虑x方向，theta为0，cmd_vel中是在world系,0912引入坐标系来了，在上面
        fg[2 + _x_start + i] = x1 - (x0 + vx_map / w_map);
        fg[2 + _y_start + i] = y1 - (y0 + vy_map /w_map);
        fg[2 + _theta_start + i] = theta1 - (theta0 + w_ * _dt);
        //cout << "DEBUG6" << endl;
    }
}