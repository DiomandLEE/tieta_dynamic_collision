#include "tieta_mpc_sim_demo/FG_evalue.h"
#include "tieta_mpc_sim_demo/MPC_Controller.h"

// #include <cppad/cppad.hpp>
#include <NLsolver/cppad/ipopt/solve.hpp>
#include <Eigen/Core>

// The program use fragments of code from
// https://github.com/udacity/CarND-MPC-Quizzes
using namespace std;
using CppAD::AD;
// ====================================
// MPC class definition implementation.
// ====================================
MPC::MPC()
{

}

MPC::MPC(Collision_Check &collision_check)
    : _collision_check(collision_check)
{
    // Set default value 也就是在默认构造函数中，给到了一些默认参数值
    _mpc_steps = 20;
    _max_angvel = 3.0;    // Maximal angvel
    _max_vel = 1.0;       // Maximal velocity
    _bound_value = 1.0e3; // Bound value for other variables
    //base
    _angel_lower = -M_PI;
    _angel_upper = M_PI;
    //UR5e
    _joint1_lower = -M_PI;
    _joint1_upper = M_PI;
    _joint2_lower = -M_PI;
    _joint2_upper = M_PI;
    _joint3_lower = -M_PI;
    _joint3_upper = M_PI;
    _joint4_lower = -M_PI;
    _joint4_upper = M_PI;
    _joint5_lower = -M_PI;
    _joint5_upper = M_PI;
    _joint6_lower = -M_PI;
    _joint6_upper = M_PI;

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
    //! e.g.如果mpc_step是10，那么求解的所有变量维度就是 (90 + 81 = 171) * 1

    _file_path_class_MPC = " ";
    _file_debug_path_class_MPC = " ";
    _file_debug_path_class_FG_eval = " ";
}

void MPC::LoadParams(const std::map<string, double> &params)
{
    //!这里在MPCNode的构造函数中，就把参数传进去了
    _params = params;
    // Init parameters for MPC object
    _mpc_steps = _params.find("STEPS") != _params.end() ? _params.at("STEPS") : _mpc_steps;
    _max_vel = _params.find("MAXVEL") != _params.end() ? _params.at("MAXVEL") : _max_vel;
    _max_angvel = _params.find("MAX_ANGVEL") != _params.end() ? _params.at("MAX_ANGVEL") : _max_angvel;
    _bound_value = _params.find("BOUND") != _params.end() ? _params.at("BOUND") : _bound_value;
    //底盘的theta上下界
    _angel_upper = _params.find("ANGEL_UPPER") != _params.end() ? _params.at("ANGEL_UPPER") : _angel_upper;
    _angel_lower = _params.find("ANGEL_LOWER") != _params.end() ? _params.at("ANGEL_LOWER") : _angel_lower;
    //机械臂UR的上下限
    _joint1_upper  = _params.find("JOINT1_UPPER") != _params.end()  ? _params.at("JOINT1_UPPER") : _joint1_upper;
    _joint1_lower  = _params.find("JOINT1_LOWER") != _params.end()  ? _params.at("JOINT1_LOWER") : _joint1_lower;
    _joint2_upper  = _params.find("JOINT2_UPPER") != _params.end()  ? _params.at("JOINT2_UPPER") : _joint2_upper;
    _joint2_lower  = _params.find("JOINT2_LOWER") != _params.end()  ? _params.at("JOINT2_LOWER") : _joint2_lower;
    _joint3_upper  = _params.find("JOINT3_UPPER") != _params.end()  ? _params.at("JOINT3_UPPER") : _joint3_upper;
    _joint3_lower  = _params.find("JOINT3_LOWER") != _params.end()  ? _params.at("JOINT3_LOWER") : _joint3_lower;
    _joint4_upper  = _params.find("JOINT4_UPPER") != _params.end()  ? _params.at("JOINT4_UPPER") : _joint4_upper;
    _joint4_lower  = _params.find("JOINT4_LOWER") != _params.end()  ? _params.at("JOINT4_LOWER") : _joint4_lower;
    _joint5_upper  = _params.find("JOINT5_UPPER") != _params.end()  ? _params.at("JOINT5_UPPER") : _joint5_upper;
    _joint5_lower  = _params.find("JOINT5_LOWER") != _params.end()  ? _params.at("JOINT5_LOWER") : _joint5_lower;
    _joint6_upper  = _params.find("JOINT6_UPPER") != _params.end()  ? _params.at("JOINT6_UPPER") : _joint6_upper;
    _joint6_lower  = _params.find("JOINT6_LOWER") != _params.end()  ? _params.at("JOINT6_LOWER") : _joint6_lower;

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
    _jntvel1_start = _angvel_start + _mpc_steps - 1;
    _jntvel2_start = _jntvel1_start + _mpc_steps - 1;
    _jntvel3_start = _jntvel2_start + _mpc_steps - 1;
    _jntvel4_start = _jntvel3_start + _mpc_steps - 1;
    _jntvel5_start = _jntvel4_start + _mpc_steps - 1;
    _jntvel6_start = _jntvel5_start + _mpc_steps - 1;

    const std::string filename = _file_path_class_MPC + "/prediction_traj.csv";
    file = std::ofstream(filename, ios::app);
    // file.open(filename); //!上面那行代码,默认就自动打开文件了, 不用加这一行代码, 不然就会得到的是空文件夹.
    if (file.is_open())
        ROS_INFO("Class mpc : record_csv/csv file has been open !");
    else
        ROS_ERROR("Class mpc : Cannot open record file %s", filename.c_str());
    file << "000_pos_map,001robot_vel,011world_vel" << endl;

    const std::string filename_debug = _file_debug_path_class_MPC + "/debug_prediction_traj.csv";
    file_debug = std::ofstream(filename_debug, ios::app);
    // file_debug.open(filename_debug);
    if (file_debug.is_open())
        ROS_INFO("Class mpc : debug_csv/csv file has been open !");
    else
        ROS_ERROR("Class mpc : Cannot open debug file: %s", filename_debug.c_str());

    cout << "\n!! MPC Obj parameters updated !! " << endl;
}

vector<double> MPC::Solve(Eigen::VectorXd state, JointTrajPub::AnglesList trackTraj, vector<Eigen::Vector3d> tf_state, bool _terminal_flag, int _terminal_nums)
{
    cout << "\n!! MPC Obj Solve Called !! " << endl;
    bool ok = true;
    size_t i;
    typedef CPPAD_TESTVECTOR(double) Dvector; // CppAD::vector< T >
    // 考虑[x;y;theta]状态，输入为[vx;vy;w]
    const double x = state[0];
    const double y = state[1];
    const double theta = state[2];
    const double jntpos1 = state[3];
    const double jntpos2 = state[4];
    const double jntpos3 = state[5];
    const double jntpos4 = state[6];
    const double jntpos5 = state[7];
    const double jntpos6 = state[8];

    const double vx = state[9];
    const double vy = state[10];
    const double angvel = state[11];
    const double jntvel1 = state[12];
    const double jntvel2 = state[13];
    const double jntvel3 = state[14];
    const double jntvel4 = state[15];
    const double jntvel5 = state[16];
    const double jntvel6 = state[17];

    //! Set the number of model variables (includes both states and inputs).
    size_t state_num = 9;  // state.size(); 3+6
    size_t input_num = 9;
    // 设置优化变量
    size_t n_vars = _mpc_steps * state_num + (_mpc_steps - 1) * input_num;

    // Initial value of the independent variables. 自变量
    // SHOULD BE 0 besides initial state.除初始状态外，还应为 0。
    //! 这里的vars是对所有待优化计算的变量的总和，是x+u
    Dvector vars(n_vars); // 状态和输入一起放进去
    for (int i = 0; i < n_vars; i++)
    {
        vars[i] = 0;
    }

    // Set the initial state variable values
    //! 赋初值，有初值就为初值，没有的话默认为0，反正都是要优化的
    vars[_x_start] = x;
    for (int i = _x_start; i < _y_start; i++)
        vars[i] = x;
    vars[_y_start] = y;
    for(int i = _y_start; i < _theta_start; i++)
        vars[i] = y;
    vars[_theta_start] = theta;
    for(int i = _theta_start; i < _joint1_start; i++)
        vars[i] = theta;

    vars[_joint1_start] = jntpos1;
    for(int i = _joint1_start; i < _joint2_start; i++)
        vars[i] = jntpos1;
    vars[_joint2_start] = jntpos2;
    for(int i = _joint2_start; i < _joint3_start; i++)
        vars[i] = jntpos2;
    vars[_joint3_start] = jntpos3;
    for(int i = _joint3_start; i < _joint4_start; i++)
        vars[i] = jntpos3;
    vars[_joint4_start] = jntpos4;
    for(int i = _joint4_start; i < _joint5_start; i++)
        vars[i] = jntpos4;
    vars[_joint5_start] = jntpos5;
    for(int i = _joint5_start; i < _joint6_start; i++)
        vars[i] = jntpos5;
    vars[_joint6_start] = jntpos6;
    for(int i = _joint6_start; i < _vx_start; i++)
        vars[i] = jntpos6;

    //debug 一定要限制vel_start，不然mpc计算的时候，就相当于0，0，x1，x2 ...
    vars[_vx_start] = vx;
    vars[_vy_start] = vy;
    vars[_angvel_start] = angvel;

    vars[_jntvel1_start] = jntvel1;
    vars[_jntvel2_start] = jntvel2;
    vars[_jntvel3_start] = jntvel3;
    vars[_jntvel4_start] = jntvel4;
    vars[_jntvel5_start] = jntvel5;
    vars[_jntvel6_start] = jntvel6;

    //?####################### 至此赋予初值完毕 ###############################

    // Set lower and upper limits for variables.
    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);

    // Set all non-actuators upper and lowerlimits
    // to the max negative and positive values.
    for (int i = 0; i < _theta_start; i++)
    {
        vars_lowerbound[i] = -_bound_value;
        vars_upperbound[i] = _bound_value;
    } // x,y

    //!位置边界值约束 The upper and lower limits of theta are set to -pi and pi
    for (int i = _theta_start; i < _joint1_start; i++)
    {
        vars_lowerbound[i] = _angel_lower;
        vars_upperbound[i] = _angel_upper;
    } // theta
        for (int i = _joint1_start; i < _joint2_start; i++)
    {
        vars_lowerbound[i] = _joint1_lower;
        vars_upperbound[i] = _joint1_upper;
    }//joint1

    // The upper and lower limits
    for (int i = _joint2_start; i < _joint3_start; i++)
    {
        vars_lowerbound[i] = _joint2_lower;
        vars_upperbound[i] = _joint2_upper;
    }//joint2

    for (int i = _joint3_start; i < _joint4_start; i++)
    {
        vars_lowerbound[i] = _joint3_lower;
        vars_upperbound[i] = _joint3_upper;
    }//joint3

    for (int i = _joint4_start; i < _joint5_start; i++)
    {
        vars_lowerbound[i] = _joint4_lower;
        vars_upperbound[i] = _joint4_upper;
    }//joint4

    for (int i = _joint5_start; i < _joint6_start; i++)
    {
        vars_lowerbound[i] = _joint5_lower;
        vars_upperbound[i] = _joint5_upper;
    }//joint5

    for (int i = _joint6_start; i < _vx_start; i++)
    {
        vars_lowerbound[i] = _joint6_lower;
        vars_upperbound[i] = _joint6_upper;
    }//joint6


    //!速度边界值约束 Acceleration/decceleration upper and lower limits
    for (int i = _vx_start; i < _angvel_start; i++)
    {
        vars_lowerbound[i] = -_max_vel;
        vars_upperbound[i] = _max_vel;
    } // vx,vy

    // The upper and lower limits of angvel
    for (int i = _angvel_start; i < n_vars; i++)
    {
        vars_lowerbound[i] = -_max_angvel;
        vars_upperbound[i] = _max_angvel;
    }

    //! 决策变量设置约束：只对初始值设置约束
    //! 位置强制初值
    vars_lowerbound[_x_start] = x;
    vars_upperbound[_x_start] = x;

    vars_lowerbound[_y_start] = y;
    vars_upperbound[_y_start] = y;

    vars_lowerbound[_theta_start] = theta;
    vars_upperbound[_theta_start] = theta;

    vars_lowerbound[_joint1_start] = jntpos1;
    vars_upperbound[_joint1_start] = jntpos1;

    vars_lowerbound[_joint2_start] = jntpos2;
    vars_upperbound[_joint2_start] = jntpos2;

    vars_lowerbound[_joint3_start] = jntpos3;
    vars_upperbound[_joint3_start] = jntpos3;

    vars_lowerbound[_joint4_start] = jntpos4;
    vars_upperbound[_joint4_start] = jntpos4;

    vars_lowerbound[_joint5_start] = jntpos5;
    vars_upperbound[_joint5_start] = jntpos5;

    vars_lowerbound[_joint6_start] = jntpos6;
    vars_upperbound[_joint6_start] = jntpos6;

    //!速度强制初值
    vars_lowerbound[_vx_start] = vx;
    vars_upperbound[_vx_start] = vx;

    vars_lowerbound[_vy_start] = vy;
    vars_upperbound[_vy_start] = vy;

    vars_lowerbound[_angvel_start] = angvel;
    vars_upperbound[_angvel_start] = angvel;

    vars_lowerbound[_jntvel1_start] = jntvel1;
    vars_upperbound[_jntvel1_start] = jntvel1;

    vars_lowerbound[_jntvel2_start] = jntvel2;
    vars_upperbound[_jntvel2_start] = jntvel2;

    vars_lowerbound[_jntvel3_start] = jntvel3;
    vars_upperbound[_jntvel3_start] = jntvel3;

    vars_lowerbound[_jntvel4_start] = jntvel4;
    vars_upperbound[_jntvel4_start] = jntvel4;

    vars_lowerbound[_jntvel5_start] = jntvel5;
    vars_upperbound[_jntvel5_start] = jntvel5;

    vars_lowerbound[_jntvel6_start] = jntvel6;
    vars_upperbound[_jntvel6_start] = jntvel6;
    //?####################### 至此决策变量的约束设置完毕 ###########################################

    //! Set the number of constraints
    //! 这里的约束是对状态量x的约束，只是对x本身的
    size_t n_constraints = _mpc_steps * state_num;
    // Lower and upper limits for the constraints
    // Should be 0 besides initial state.
    //! 状态量的约束
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);

    for (int i = 0; i < n_constraints; i++)
    {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
        //! 必须是0,g[1]是初始约束，g[2+0]~g[2+_mpc_step-2]是状态方程的个数，1+(_mpc_step-1)->_mpc_step
    }
    constraints_lowerbound[_x_start] = x;
    constraints_lowerbound[_y_start] = y;
    constraints_lowerbound[_theta_start] = theta;
    constraints_lowerbound[_joint1_start] = jntpos1;
    constraints_lowerbound[_joint2_start] = jntpos2;
    constraints_lowerbound[_joint3_start] = jntpos3;
    constraints_lowerbound[_joint4_start] = jntpos4;
    constraints_lowerbound[_joint5_start] = jntpos5;
    constraints_lowerbound[_joint6_start] = jntpos6;

    constraints_upperbound[_x_start] = x;
    constraints_upperbound[_y_start] = y;
    constraints_upperbound[_theta_start] = theta;
    constraints_upperbound[_joint1_start] = jntpos1;
    constraints_upperbound[_joint2_start] = jntpos2;
    constraints_upperbound[_joint3_start] = jntpos3;
    constraints_upperbound[_joint4_start] = jntpos4;
    constraints_upperbound[_joint5_start] = jntpos5;
    constraints_upperbound[_joint6_start] = jntpos6;

    //? ################### 至此，硬约束函数构建完毕，没有不等式约束，都是等式约束 #####################
    //? ### x0 = map_x_0 && 0 = x1 - (x0 + vx_0 * dt) && 这个运动学方程有mpc_steps - 1个，加上第一个初始约束，一维上就有mpc_Steps个约束 ###

    //! 计算目标和约束 object that computes objective and constraints： f[0]代表object function，f[1]开始代表g(x)约束
    FG_eval fg_eval = FG_eval(trackTraj, _collision_check, tf_state, _terminal_flag, _terminal_nums); //实例化 一个FG_eval的类

    //------ 文件夹的命名 ------
    fg_eval._file_debug_path = _file_debug_path_class_FG_eval + "/fg_evalue_segTraj.csv";
    // fg_eval.file_debug = std::ofstream(fg_eval._file_debug_path);
    fg_eval.file_debug.open(fg_eval._file_debug_path, ios::app);
    if (fg_eval.file_debug.is_open())
        ROS_INFO("Class FG_eval : debug_csv/csv file has been open ! : %s", fg_eval._file_debug_path.c_str());
    else
        ROS_ERROR("Class FG_eval : Cannot open debug file: %s", fg_eval._file_debug_path.c_str());

    fg_eval.LoadParams(_params); // MPC类中的_params
    // 就是加载个系数...

    // // options for IPOPT solver
    // std::string options;
    // // Uncomment this if you'd like more print information如果您想了解更多打印信息，请取消注释
    // //! 开启注释
    // // options += "Integer print_level  0\n";
    // //  NOTE: Setting sparse to true allows the solver to take advantage
    // //  of sparse routines, this makes the computation MUCH FASTER. If you
    // //  can uncomment 1 of these and see if it makes a difference or not but
    // //  if you uncomment both the computation time should go up in orders of
    // //  magnitude.
    // //  注意：将稀疏设置为 true 允许求解器利用稀疏例程，这使得计算速度更快。
    // options += "Sparse  true        forward\n";
    // options += "Sparse  true        reverse\n";
    // // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
    // // Change this as you see fit.// 注意：目前x direction distance error:求解器的最大时间限制为 0.5 秒。
    // // 根据您的需要更改此设置。
    // options += "Numeric max_cpu_time          0.5\n";
    //!new options
    std::string options;
    // turn off any printing 注释下面这一行将关闭CPPAD的求解过程的LOG
    // options += "Integer print_level  0\n";
    // options += "String  sb           yes\n";
    // maximum number of iterations
    options += "Integer max_iter     30\n";
    // approximate accuracy in first order necessary conditions;
    // see Mathematical Programming, Volume 106, Number 1,
    // Pages 25-57, Equation (6)
    options += "Numeric tol          1e-12\n";
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    //options += "Numeric max_cpu_time          0.05\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;
    cout << "+++++++++++++++++  开始求解  +++++++++++++++" << endl;
    file_debug << "+++++++++++++++++  开始求解  +++++++++++++++" << endl;

    std::cout << "cppAd input size:" << std::endl;
    std::cout << "n_var:" << vars.size() << std::endl;
    std::cout << "n_var_lowb" << vars_lowerbound.size() << std::endl;
    std::cout << "n_var_upb" << vars_upperbound.size() << std::endl;
    std::cout << "n_con_lowb:" << constraints_lowerbound.size() << std::endl;
    std::cout << "n_con_upb:" << constraints_upperbound.size() << std::endl;

    // solve the problem 返回解决方案的地方
    CppAD::ipopt::solve<Dvector, FG_eval>(
        options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
        constraints_upperbound, fg_eval, solution);

    vars = solution.x;
    CppAD::ipopt::solve<Dvector, FG_eval>(
        options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
        constraints_upperbound, fg_eval, solution);

    fg_eval.file_debug.close();

    //-------  这里是想每一个预测文件都保存一个文件 -------------
    /*
    // todo //done
    // _loop_cnt = loop_cnt;
    // const std::string filename_temp = _file_path_class_MPC + "/prediction_traj_loop_" + to_string(_loop_cnt) + ".csv";
    // file = std::ofstream(filename_temp,ios::app);
    // //file.open(filename_temp);
    // if(file.is_open())
    //     ROS_INFO("Class mpc : record_csv/csv file has been open !");
    // else
    //     ROS_ERROR("Class mpc : Cannot open record file %s", filename_temp.c_str());
    //-------------------------------------------------------
    */

    cout << "+++++++++++++++++  结束求解  ++++++++++++++++" << endl;
    file_debug << "+++++++++++++++++  结束求解  ++++++++++++++++" << endl;

    // Check some of the solution values
    // C++运算中的优先级顺序，比较运算 优于 赋值运算 https://haicoder.net/cpp/cpp-operator-priority.html
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

    // Cost
    auto cost = solution.obj_value; //! return fg[0]

    cout << "$$$$$$$$$ 结果展示 $$$$$$$$$$" << endl;
    std::cout << "------ Total Cost(solution): " << cost << "-----" << std::endl;
    // fg_value类的只有x_cost的变量
    // _mpc_distx_Tcost = CppAD::Value(fg_eval.cost_distx);
    // _mpc_disty_Tcost = CppAD::Value(fg_eval.cost_disty);
    // _mpc_etheta_Tcost = CppAD::Value(fg_eval.cost_etheta);


    this->mpc_x = {};
    this->mpc_y = {};
    this->mpc_theta = {};
    this->mpc_joint1 = {};
    this->mpc_joint2 = {};
    this->mpc_joint3 = {};
    this->mpc_joint4 = {};
    this->mpc_joint5 = {};
    this->mpc_joint6 = {};

    cout << "------ solution结果: " << ok << "------" << endl;

    // file_debug << "------ solution结果: " << ok << "------" << endl;
    // file << "0,0,0" << endl; //for predicition position solution

    cout << "------ solution,size(): " << solution.x.size() << "-----" << endl;

    // predict result，[0] for start_init ,{1-**} for prediction
    for (int i = 0; i < _mpc_steps - 1; i++)
    {
        // print 执行前一时刻的速度和加速度 得到的当前时刻的位姿
        // cout << "----- world系位置预测结果 "
        //      << "x_" << i << ":  " << solution.x[_x_start + i] << " , "
        //      << "y_" << i << ":  " << solution.x[_y_start + i] << " , "
        //      << "theta_" << i << ":  " << solution.x[_theta_start + i] << " -----" << endl;
        // file << solution.x[_x_start + i] << "," << solution.x[_y_start + i] << "," << solution.x[_theta_start + i] << endl;

        // // print 前一时刻的速度和角速度
        // cout << "----- ROBOT系速度预测结果 "
        //      << "vx_rb_" << i << ":  " << solution.x[_vx_start + i] << " , "
        //      << "vy_rb_" << i << ":  " << solution.x[_vy_start + i] << " , "
        //      << "omega_rb_" << i << ":  " << solution.x[_angvel_start + i] << " ----" << endl;

        // cout << "----- WORLD系速度预测结果 "
        //      << "vx_" << i << ":  " << CppAD::cos(solution.x[_theta_start + i]) * solution.x[_vx_start + i] - CppAD::sin(solution.x[_theta_start + i]) * solution.x[_vy_start + i] << " , "
        //      << "vy_" << i << ":  " << CppAD::sin(solution.x[_theta_start + i]) * solution.x[_vx_start + i] + CppAD::cos(solution.x[_theta_start + i]) * solution.x[_vy_start + i] << " , "
        //      << "omega_" << i << ":  " << solution.x[_angvel_start + i] << " ----" << endl;

        // 预测结果装在mpc_state[],
        this->mpc_x.push_back(solution.x[_x_start + i + 1]);
        this->mpc_y.push_back(solution.x[_y_start + i + 1]);
        this->mpc_theta.push_back(solution.x[_theta_start + i + 1]);

        this->mpc_joint1.push_back(solution.x[_joint1_start + i + 1]);
        this->mpc_joint2.push_back(solution.x[_joint2_start + i + 1]);
        this->mpc_joint3.push_back(solution.x[_joint3_start + i + 1]);
        this->mpc_joint4.push_back(solution.x[_joint4_start + i + 1]);
        this->mpc_joint5.push_back(solution.x[_joint5_start + i + 1]);
        this->mpc_joint6.push_back(solution.x[_joint6_start + i + 1]);

        // //print 执行前一时刻的速度和加速度 得到的当前时刻的位姿
        // cout << "----- 位置预测结果 " <<
        //     "x_" << i << ":  " << solution.x[_x_start + i] << " , " <<
        //     "y_" << i << ":  " << solution.x[_y_start + i] << " , " <<
        //     "theta_" << i << ":  " << solution.x[_theta_start + i] << " -----" << endl;

        // this->mpc_y.push_back(solution.x[_y_start + i]);
        // print 执行前一时刻的速度和加速度 得到的当前时刻的位姿
        // file_debug << "----- world系位置预测结果 "
        //            << "x_" << i << ":  " << solution.x[_x_start + i] << " , "
        //            << "y_" << i << ":  " << solution.x[_y_start + i] << " , "
        //            << "theta_" << i << ":  " << solution.x[_theta_start + i] << " -----" << endl;

        // print 前一时刻的速度和角速度
        // file_debug << "----- ROBOT系速度预测结果 "
        //            << "vx_rb_" << i << ":  " << solution.x[_vx_start + i] << " , "
        //            << "vy_rb_" << i << ":  " << solution.x[_vy_start + i] << " , "
        //            << "omega_rb_" << i << ":  " << solution.x[_angvel_start + i] << " ----" << endl;

        // file_debug << "----- WORLD系速度预测结果 "
        //            << "vx_" << i << ":  " << CppAD::cos(solution.x[_theta_start + i]) * solution.x[_vx_start + i] - CppAD::sin(solution.x[_theta_start + i]) * solution.x[_vy_start + i] << " , "
        //            << "vy_" << i << ":  " << CppAD::sin(solution.x[_theta_start + i]) * solution.x[_vx_start + i] + CppAD::cos(solution.x[_theta_start + i]) * solution.x[_vy_start + i] << " , "
        //            << "omega_" << i << ":  " << solution.x[_angvel_start + i] << " ----" << endl;
    }

    //这里是往file里面存放数据
    /*
    // //file << "----------------  end  ---------------" << endl;
    // file_debug << "\n" << endl;

    // file << "0,0,1" << endl; //for  predicition velocity_robot solution

    // for (int i = 0; i < _mpc_steps - 1; i++)
    // {
    //     // print 执行前一时刻的速度和加速度 得到的当前时刻的位姿
    //     file << solution.x[_vx_start + i] << "," << solution.x[_vy_start + i] << ","
    //             << solution.x[_angvel_start + i] << endl;
    // }
    // file << "----------------  end  ---------------" << endl;
    // file << "\n" << endl;

    //file << "0,1,1" << endl; //for predicition velocity_world solution

    // for (int i = 0; i < _mpc_steps - 1; i++)
    // {
    //     // print 执行前一时刻的速度和加速度 得到的当前时刻的位姿
    //     file << CppAD::cos(solution.x[_theta_start + i]) * solution.x[_vx_start + i] - CppAD::sin(solution.x[_theta_start + i]) * solution.x[_vy_start + i] << ","
    //             << CppAD::sin(solution.x[_theta_start + i]) * solution.x[_vx_start + i] + CppAD::cos(solution.x[_theta_start + i]) * solution.x[_vy_start + i] << ","
    //             << solution.x[_angvel_start + i] << endl;
    // }
    // file << "----------------  end  ---------------" << endl;
    // file << "\n" << endl;
    // file.close(); //每次预测单独保存
    */
    //debug 根本不需要确定10啊，这后面push back不就成size=20了么。。。
    vector<double> result; //九个自由度的速度控制 + 判断flag（如果这次没有解出来的话，就停在原地，并且下次的跟踪目标，仍然为这次loop的目标）

    if(ok)
    {
        std::cout << "$$$$$$ MPC solved successfully! $$$$$$" << std::endl;
        //read 如果MPC求解出来了结果，那么机器人执行这个结果
        // velocity & omaga -> return variables
        //! 更改代码后，第一个解变成了初始的机器人状态，所以给cmd_vel第二个速度指令
        result.push_back(solution.x[_vx_start + 1]);
        result.push_back(solution.x[_vy_start + 1]);
        result.push_back(solution.x[_angvel_start + 1]);

        result.push_back(solution.x[_jntvel1_start + 1]);
        result.push_back(solution.x[_jntvel2_start + 1]);
        result.push_back(solution.x[_jntvel3_start + 1]);
        result.push_back(solution.x[_jntvel4_start + 1]);
        result.push_back(solution.x[_jntvel5_start + 1]);
        result.push_back(solution.x[_jntvel6_start + 1]);

        result.push_back(1.0); // 判断flag
        for(int i = 0; i < 9; i++)
            std::cout << result[i] << ",";
        std::cout << result[9] <<  std::endl;
    }
    else
        result = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    return result;
}
