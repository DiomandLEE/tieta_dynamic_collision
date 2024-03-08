#include "open_door_tieta_mpc/FG_evalue.h"
//#include <cppad/cppad.hpp>
//#include <NLsolver/cppad/ipopt/solve.hpp>
#include <Eigen/Core>
typedef CPPAD_TESTVECTOR(double) Dvector;
typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
using CppAD::AD;
//! barried func是在这里应用的

// =========================================
// FG_eval class definition implementation.
// =========================================
FG_eval::FG_eval()
{}
//构造函数---赋初值
FG_eval::FG_eval(DoorTrajPub::AnglesList _trackTraj, vector<Eigen::Vector3d> tf_state, bool _flag, int _nums)
    : _mpc_trackTraj(_trackTraj), _init_sphere(tf_state), _terminal_flag(_flag), _terminal_nums(_nums)
{
    //采样时间和控制率频率
    _dt = 0.1; //sec
    _w_distx     = 100;
    _w_disty     = 100;
    _w_etheta    = 100;
    _w_vel     = 1;
    _w_angvel   = 100;

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


    //9个速度变量，(9 * (mpc_step - 1)) * 1
    _vx_start = _theta_start + _mpc_steps;
    _vy_start = _vx_start + _mpc_steps - 1;
    _angvel_start = _vy_start + _mpc_steps - 1;


    //? 来看一下，计算误差用到的position
    cost_distx = 0;
    cost_disty = 0;
    cost_etheta = 0;

    cost_vx = 0;
    cost_vy = 0;
    cost_angvel = 0;


    //_terminal_flag = false;
    //_terminal_nums = _mpc_steps - 1;

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

    _w_acc = params.find("W_ACC") != params.end()     ? params.at("W_ACC") : _w_acc;
    _w_angacc = params.find("W_ANGACC") != params.end() ? params.at("W_ANGACC") : _w_angacc;

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

    //for sigmod
    _barried_func_arm_n = params.find("BARRIED_ARM_n") != params.end() ? params.at("BARRIED_ARM_n") : _barried_func_arm_n;
    _barried_func_arm_w = params.find("BARRIED_ARM_w") != params.end() ? params.at("BARRIED_ARM_w") : _barried_func_arm_w;
    _barried_func_arm_m = params.find("BARRIED_ARM_m") != params.end() ? params.at("BARRIED_ARM_m") : _barried_func_arm_m;
    _barried_func_arm_r = params.find("BARRIED_ARM_r") != params.end() ? params.at("BARRIED_ARM_r") : _barried_func_arm_r;

    _barried_func_base_n = params.find("BARRIED_BASE_n") != params.end() ? params.at("BARRIED_BASE_n") : _barried_func_base_n;
    _barried_func_base_w = params.find("BARRIED_BASE_w") != params.end() ? params.at("BARRIED_BASE_w") : _barried_func_base_w;
    _barried_func_base_m = params.find("BARRIED_BASE_m") != params.end() ? params.at("BARRIED_BASE_m") : _barried_func_base_m;
    _barried_func_base_r = params.find("BARRIED_BASE_r") != params.end() ? params.at("BARRIED_BASE_r") : _barried_func_base_r;

    //state index
    //9个位置变量，(9 * mpc_step) * 1
    _x_start = 0;
    _y_start = _x_start + _mpc_steps;
    _theta_start = _y_start + _mpc_steps;

    //9个速度变量，(9 * (mpc_step - 1)) * 1
    _vx_start = _vx_start + _mpc_steps;
    _vy_start = _vx_start + _mpc_steps - 1;
    _angvel_start = _vy_start + _mpc_steps - 1;

    //todo 不需要终端约束了
    // //set mec_steps _w_hard_ee_tool[]
    // for (int w = 0; w < _mpc_steps; w++){
    //     _w_vector_terminal.push_back(0.0);
    // }//设置了 终端约束的大小

    // cout << "terminal_constraint weights: ";

    // if(_terminal_flag){
    //     for (int w = 0; w < _mpc_steps; w++){
    //         if(w >= _terminal_nums)
    //             _w_vector_terminal[w] = _w_hard_EE_tool;
    //         cout << _w_vector_terminal[w] << ",";
    //     }
    // }
    // cout << endl;


        std::cout << "FG_eval paramster has been set." << std::endl;
}

    //barried func
AD<double> FG_eval::barried_func_arm_(AD<double> distance_)
{
    distance_ = _barried_func_arm_w / (_barried_func_arm_n + CppAD::exp(_barried_func_arm_m * (distance_ - _barried_func_arm_r)));
    return distance_;
}
AD<double> FG_eval::barried_func_base_(AD<double> distance_)
{
    distance_ = _barried_func_base_w / (_barried_func_base_n + CppAD::exp(_barried_func_base_m * (distance_ - _barried_func_base_r)));
    return distance_;
}

// forwards kinematics 解析式公式
int FG_eval::casadi_wrist_sphere(ADvector arg, ADvector& res) {
    //read here arg.size() = 7:
    //['base_y_base_x', 'base_theta_base_y', 'base_link_base_theta', 'right_arm_shoulder_pan_joint', 'right_arm_shoulder_lift_joint', 'right_arm_elbow_joint', 'right_arm_wrist_1_joint']
    AD<double> a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a3, a4, a5, a6, a7, a8, a9;
    a0=1.2246467991473532e-16;
    a1 = arg[2]; // a1=arg[0]? arg[0][2] : 0;
    a2=cos(a1);
    a3=6.1232339957367660e-17;
    a4=-7.0710678118654746e-01;
    a1=sin(a1);
    a5=(a4*a1);
    a6=(a3*a5);
    a6=(a2-a6);
    a7=-6.1232339957367660e-17;
    a8=7.0710678118654757e-01;
    a9=(a8*a1);
    a10=(a7*a9);
    a6=(a6-a10);
    a10=(a0*a6);
    a11=(a3*a2);
    a11=(a11-a9);
    a10=(a10-a11);
    a12 = arg[3]; // a12=arg[0]? arg[0][3] : 0;
    a13=cos(a12);
    a14=(a10*a13);
    a15=-1.2246467991473532e-16;
    a11=(a15*a11);
    a11=(a11-a6);
    a12=sin(a12);
    a6=(a11*a12);
    a14=(a14+a6);
    a6 = arg[4]; // a6=arg[0]? arg[0][4] : 0;
    a16=cos(a6);
    a17=(a14*a16);
    a11=(a11*a13);
    a10=(a10*a12);
    a11=(a11-a10);
    a10=-2.0510342851533115e-10;
    a6=sin(a6);
    a18=(a10*a6);
    a19=(a11*a18);
    a17=(a17+a19);
    a19=(a7*a2);
    a19=(a19-a5);
    a5=3.7493994566546440e-33;
    a9=(a5*a9);
    a19=(a19-a9);
    a9=(a19*a6);
    a17=(a17+a9);
    a9 = arg[5]; // a9=arg[0]? arg[0][5] : 0;
    a20=cos(a9);
    a21=(a17*a20);
    a22=(a10*a16);
    a23=(a11*a22);
    a14=(a14*a6);
    a23=(a23-a14);
    a14=(a19*a16);
    a23=(a23+a14);
    a9=sin(a9);
    a14=(a23*a9);
    a21=(a21+a14);
    a14 = arg[6]; // a14=arg[0]? arg[0][6] : 0;
    a24=cos(a14);
    a25=(a21*a24);
    a23=(a23*a20);
    a26=(a17*a9);
    a23=(a23-a26);
    a14=sin(a14);
    a26=(a23*a14);
    a25=(a25+a26);
    res[0] = a25; // if (res[0]!=0) res[0][0]=a25;
    a4=(a4*a2);
    a25=(a3*a4);
    a25=(a1+a25);
    a8=(a8*a2);
    a26=(a7*a8);
    a25=(a25+a26);
    a0=(a0*a25);
    a3=(a3*a1);
    a3=(a3+a8);
    a0=(a0-a3);
    a26=(a0*a13);
    a15=(a15*a3);
    a15=(a15-a25);
    a25=(a15*a12);
    a26=(a26+a25);
    a25=(a26*a16);
    a15=(a15*a13);
    a0=(a0*a12);
    a15=(a15-a0);
    a0=(a15*a18);
    a25=(a25+a0);
    a7=(a7*a1);
    a7=(a7+a4);
    a5=(a5*a8);
    a7=(a7+a5);
    a5=(a7*a6);
    a25=(a25+a5);
    a5=(a25*a20);
    a8=(a15*a22);
    a26=(a26*a6);
    a8=(a8-a26);
    a26=(a7*a16);
    a8=(a8+a26);
    a26=(a8*a9);
    a5=(a5+a26);
    a26=(a5*a24);
    a8=(a8*a20);
    a4=(a25*a9);
    a8=(a8-a4);
    a4=(a8*a14);
    a26=(a26+a4);
    res[1] = a26; // if (res[0]!=0) res[0][1]=a26;
    a26=7.0710678118654746e-01;
    a4=(a26*a13);
    a0=8.6595605623549329e-17;
    a3=(a0*a12);
    a4=(a4+a3);
    a3=(a4*a16);
    a0=(a0*a13);
    a26=(a26*a12);
    a0=(a0-a26);
    a18=(a0*a18);
    a3=(a3+a18);
    a18=-7.0710678118654757e-01;
    a26=(a18*a6);
    a3=(a3+a26);
    a26=(a3*a20);
    a22=(a0*a22);
    a4=(a4*a6);
    a22=(a22-a4);
    a18=(a18*a16);
    a22=(a22+a18);
    a18=(a22*a9);
    a26=(a26+a18);
    a18=(a26*a24);
    a22=(a22*a20);
    a9=(a3*a9);
    a22=(a22-a9);
    a9=(a22*a14);
    a18=(a18+a9);
    res[2] = a18; // if (res[0]!=0) res[0][2]=a18;
    a18=0.;
    res[3] = a18; //if (res[0] != 0) res[0][3] = a18;
    a23=(a23*a24);
    a9=(a21*a14);
    a23=(a23-a9);
    res[4] = a23; // if (res[0]!=0) res[0][4]=a23;
    a8=(a8*a24);
    a9=(a5*a14);
    a8=(a8-a9);
    res[5] = a8; // if (res[0]!=0) res[0][5]=a8;
    a22=(a22*a24);
    a14=(a26*a14);
    a22=(a22-a14);
    res[6] = a22; // if (res[0]!=0) res[0][6]=a22;
    res[7] = a18; // if (res[0]!=0) res[0][7]=a18;
    a14=(a10*a19);
    a14=(a14-a11);
    res[8] = a14; // if (res[0]!=0) res[0][8]=a14;
    a10=(a10*a7);
    a10=(a10-a15);
    res[9] = a10; // if (res[0]!=0) res[0][9]=a10;
    a15=1.4503002514780097e-10;
    a15=(a15-a0);
    res[10] = a15; // if (res[0]!=0) res[0][10]=a15;
    res[11] = a18; // if (res[0]!=0) res[0][11]=a18;
    a18=-4.0000000000000001e-02;
    a23=(a18*a23);
    a0=-1.0000000000000000e-02;
    a11=(a0*a14);
    a23=(a23+a11);
    a11=-3.9219999999999999e-01;
    a21=(a11*a21);
    a24=1.3330000000000000e-01;
    a14=(a24*a14);
    a21=(a21+a14);
    a14=-4.2499999999999999e-01;
    a17=(a14*a17);
    a9=1.6250000000000001e-01;
    a19=(a9*a19);
    a20=2.0000000000000001e-01;
    a16=(a20*a2);
    a4=-2.2000000000000000e-01;
    a6=(a4*a1);
    a16=(a16-a6);
    a6 = arg[0]; // a6=arg[0]? arg[0][0] : 0;
    a16=(a16+a6);
    a19=(a19+a16);
    a17=(a17+a19);
    a21=(a21+a17);
    a23=(a23+a21);
    res[12] = a23; // if (res[0]!=0) res[0][12]=a23;
    a8=(a18*a8);
    a23=(a0*a10);
    a8=(a8+a23);
    a5=(a11*a5);
    a10=(a24*a10);
    a5=(a5+a10);
    a25=(a14*a25);
    a9=(a9*a7);
    a20=(a20*a1);
    a4=(a4*a2);
    a20=(a20+a4);
    a4 = arg[1]; // a4=arg[0]? arg[0][1] : 0;
    a20=(a20+a4);
    a9=(a9+a20);
    a25=(a25+a9);
    a5=(a5+a25);
    a8=(a8+a5);
    res[13] = a8; // if (res[0]!=0) res[0][13]=a8;
    a18=(a18*a22);
    a0=(a0*a15);
    a18=(a18+a0);
    a11=(a11*a26);
    a24=(a24*a15);
    a11=(a11+a24);
    a14=(a14*a3);
    a3=1.1180951480571861e+00;
    a14=(a14+a3);
    a11=(a11+a14);
    a18=(a18+a11);
    res[14] = a18; // if (res[0]!=0) res[0][14]=a18;
    a18=1.;
    res[15] = a18; // if (res[0]!=0) res[0][15]=a18;
    return 0;
}
int FG_eval::casadi_shoulder_sphere(ADvector arg, ADvector& res)
{
    /* T_fk:(i0[5])->(o0[4x4]) */
    //read arg.size():5
    //['base_y_base_x', 'base_theta_base_y', 'base_link_base_theta', 'right_arm_shoulder_pan_joint', 'right_arm_shoulder_lift_joint']
    AD<double> a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a3, a4, a5, a6, a7, a8, a9;
    a0=1.2246467991473532e-16;
    a1 = arg[2]; // a1=arg[0]? arg[0][2] : 0;
    a2=cos(a1);
    a3=6.1232339957367660e-17;
    a4=-7.0710678118654746e-01;
    a1=sin(a1);
    a5=(a4*a1);
    a6=(a3*a5);
    a6=(a2-a6);
    a7=-6.1232339957367660e-17;
    a8=7.0710678118654757e-01;
    a9=(a8*a1);
    a10=(a7*a9);
    a6=(a6-a10);
    a10=(a0*a6);
    a11=(a3*a2);
    a11=(a11-a9);
    a10=(a10-a11);
    a12 = arg[3]; // a12=arg[0]? arg[0][3] : 0;
    a13=cos(a12);
    a14=(a10*a13);
    a15=-1.2246467991473532e-16;
    a11=(a15*a11);
    a11=(a11-a6);
    a12=sin(a12);
    a6=(a11*a12);
    a14=(a14+a6);
    a6 = arg[4]; // a6=arg[0]? arg[0][4] : 0;
    a16=cos(a6);
    a17=(a14*a16);
    a11=(a11*a13);
    a10=(a10*a12);
    a11=(a11-a10);
    a10=-2.0510342851533115e-10;
    a6=sin(a6);
    a18=(a10*a6);
    a19=(a11*a18);
    a17=(a17+a19);
    a19=(a7*a2);
    a19=(a19-a5);
    a5=3.7493994566546440e-33;
    a9=(a5*a9);
    a19=(a19-a9);
    a9=(a19*a6);
    a17=(a17+a9);
    res[0] = a17; // if (res[0]!=0) res[0][0]=a17;
    a4=(a4*a2);
    a17=(a3*a4);
    a17=(a1+a17);
    a8=(a8*a2);
    a9=(a7*a8);
    a17=(a17+a9);
    a0=(a0*a17);
    a3=(a3*a1);
    a3=(a3+a8);
    a0=(a0-a3);
    a9=(a0*a13);
    a15=(a15*a3);
    a15=(a15-a17);
    a17=(a15*a12);
    a9=(a9+a17);
    a17=(a9*a16);
    a15=(a15*a13);
    a0=(a0*a12);
    a15=(a15-a0);
    a0=(a15*a18);
    a17=(a17+a0);
    a7=(a7*a1);
    a7=(a7+a4);
    a5=(a5*a8);
    a7=(a7+a5);
    a5=(a7*a6);
    a17=(a17+a5);
    res[1] = a17; // if (res[0]!=0) res[0][1]=a17;
    a17=7.0710678118654746e-01;
    a5=(a17*a13);
    a8=8.6595605623549329e-17;
    a4=(a8*a12);
    a5=(a5+a4);
    a4=(a5*a16);
    a8=(a8*a13);
    a17=(a17*a12);
    a8=(a8-a17);
    a18=(a8*a18);
    a4=(a4+a18);
    a18=-7.0710678118654757e-01;
    a17=(a18*a6);
    a4=(a4+a17);
    res[2] = a4; // if (res[0]!=0) res[0][2]=a4;
    a4=0.;
    res[3] = a4; // if (res[0]!=0) res[0][3]=a4;
    a17=(a10*a16);
    a12=(a11*a17);
    a14=(a14*a6);
    a12=(a12-a14);
    a14=(a19*a16);
    a12=(a12+a14);
    res[4] = a12; // if (res[0]!=0) res[0][4]=a12;
    a12=(a15*a17);
    a9=(a9*a6);
    a12=(a12-a9);
    a9=(a7*a16);
    a12=(a12+a9);
    res[5] = a12; // if (res[0]!=0) res[0][5]=a12;
    a17=(a8*a17);
    a5=(a5*a6);
    a17=(a17-a5);
    a18=(a18*a16);
    a17=(a17+a18);
    res[6] = a17; // if (res[0]!=0) res[0][6]=a17;
    res[7] = a4;  // if (res[0]!=0) res[0][7]=a4;
    a17=(a10*a19);
    a17=(a17-a11);
    res[8] = a17; // if (res[0]!=0) res[0][8]=a17;
    a10=(a10*a7);
    a10=(a10-a15);
    res[9] = a10; // if (res[0]!=0) res[0][9]=a10;
    a15=1.4503002514780097e-10;
    a15=(a15-a8);
    res[10] = a15; // if (res[0]!=0) res[0][10]=a15;
    res[11] = a4;  // if (res[0]!=0) res[0][11]=a4;
    a4=8.0000000000000002e-02;
    a17=(a4*a17);
    a8=1.6250000000000001e-01;
    a19=(a8*a19);
    a11=2.0000000000000001e-01;
    a18=(a11*a2);
    a16=-2.2000000000000000e-01;
    a5=(a16*a1);
    a18=(a18-a5);
    a5 = arg[0]; // a5=arg[0]? arg[0][0] : 0;
    a18=(a18+a5);
    a19=(a19+a18);
    a17=(a17+a19);
    res[12] = a17; // if (res[0]!=0) res[0][12]=a17;
    a10=(a4*a10);
    a8=(a8*a7);
    a11=(a11*a1);
    a16=(a16*a2);
    a11=(a11+a16);
    a16 = arg[1]; // a16=arg[0]? arg[0][1] : 0;
    a11=(a11+a16);
    a8=(a8+a11);
    a10=(a10+a8);
    res[13] = a10; // if (res[0]!=0) res[0][13]=a10;
    a4=(a4*a15);
    a15=1.1180951480571861e+00;
    a4=(a4+a15);
    res[14] = a4; // if (res[0]!=0) res[0][14]=a4;
    a4=1.;
    res[15] = a4; // if (res[0]!=0) res[0][15]=a4;
    return 0;
}
int FG_eval::casadi_elbow_sphere(ADvector arg, ADvector& res){
    /* T_fk:(i0[6])->(o0[4x4]) */
    //read arg.size():6
    //['base_y_base_x', 'base_theta_base_y', 'base_link_base_theta', 'right_arm_shoulder_pan_joint', 'right_arm_shoulder_lift_joint', 'right_arm_elbow_joint']
    AD<double> a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a3, a4, a5, a6, a7, a8, a9;
    a0=1.2246467991473532e-16;
    a1 = arg[2]; // a1=arg[0]? arg[0][2] : 0;
    a2=cos(a1);
    a3=6.1232339957367660e-17;
    a4=-7.0710678118654746e-01;
    a1=sin(a1);
    a5=(a4*a1);
    a6=(a3*a5);
    a6=(a2-a6);
    a7=-6.1232339957367660e-17;
    a8=7.0710678118654757e-01;
    a9=(a8*a1);
    a10=(a7*a9);
    a6=(a6-a10);
    a10=(a0*a6);
    a11=(a3*a2);
    a11=(a11-a9);
    a10=(a10-a11);
    a12 = arg[3]; // a12=arg[0]? arg[0][3] : 0;
    a13=cos(a12);
    a14=(a10*a13);
    a15=-1.2246467991473532e-16;
    a11=(a15*a11);
    a11=(a11-a6);
    a12=sin(a12);
    a6=(a11*a12);
    a14=(a14+a6);
    a6 = arg[4]; // a6=arg[0]? arg[0][4] : 0;
    a16=cos(a6);
    a17=(a14*a16);
    a11=(a11*a13);
    a10=(a10*a12);
    a11=(a11-a10);
    a10=-2.0510342851533115e-10;
    a6=sin(a6);
    a18=(a10*a6);
    a19=(a11*a18);
    a17=(a17+a19);
    a19=(a7*a2);
    a19=(a19-a5);
    a5=3.7493994566546440e-33;
    a9=(a5*a9);
    a19=(a19-a9);
    a9=(a19*a6);
    a17=(a17+a9);
    a9 = arg[5]; // a9=arg[0]? arg[0][5] : 0;
    a20=cos(a9);
    a21=(a17*a20);
    a22=(a10*a16);
    a23=(a11*a22);
    a14=(a14*a6);
    a23=(a23-a14);
    a14=(a19*a16);
    a23=(a23+a14);
    a9=sin(a9);
    a14=(a23*a9);
    a21=(a21+a14);
    res[0] = a21; // if (res[0]!=0) res[0][0]=a21;
    a4=(a4*a2);
    a21=(a3*a4);
    a21=(a1+a21);
    a8=(a8*a2);
    a14=(a7*a8);
    a21=(a21+a14);
    a0=(a0*a21);
    a3=(a3*a1);
    a3=(a3+a8);
    a0=(a0-a3);
    a14=(a0*a13);
    a15=(a15*a3);
    a15=(a15-a21);
    a21=(a15*a12);
    a14=(a14+a21);
    a21=(a14*a16);
    a15=(a15*a13);
    a0=(a0*a12);
    a15=(a15-a0);
    a0=(a15*a18);
    a21=(a21+a0);
    a7=(a7*a1);
    a7=(a7+a4);
    a5=(a5*a8);
    a7=(a7+a5);
    a5=(a7*a6);
    a21=(a21+a5);
    a5=(a21*a20);
    a8=(a15*a22);
    a14=(a14*a6);
    a8=(a8-a14);
    a14=(a7*a16);
    a8=(a8+a14);
    a14=(a8*a9);
    a5=(a5+a14);
    res[1] = a5; // if (res[0]!=0) res[0][1]=a5;
    a5=7.0710678118654746e-01;
    a14=(a5*a13);
    a4=8.6595605623549329e-17;
    a0=(a4*a12);
    a14=(a14+a0);
    a0=(a14*a16);
    a4=(a4*a13);
    a5=(a5*a12);
    a4=(a4-a5);
    a18=(a4*a18);
    a0=(a0+a18);
    a18=-7.0710678118654757e-01;
    a5=(a18*a6);
    a0=(a0+a5);
    a5=(a0*a20);
    a22=(a4*a22);
    a14=(a14*a6);
    a22=(a22-a14);
    a18=(a18*a16);
    a22=(a22+a18);
    a18=(a22*a9);
    a5=(a5+a18);
    res[2] = a5; // if (res[0]!=0) res[0][2]=a5;
    a5=0.;
    res[3] = a5; // if (res[0]!=0) res[0][3]=a5;
    a23=(a23*a20);
    a18=(a17*a9);
    a23=(a23-a18);
    res[4] = a23; // if (res[0]!=0) res[0][4]=a23;
    a8=(a8*a20);
    a18=(a21*a9);
    a8=(a8-a18);
    res[5] = a8; // if (res[0]!=0) res[0][5]=a8;
    a22=(a22*a20);
    a9=(a0*a9);
    a22=(a22-a9);
    res[6] = a22; //if (res[0]!=0) res[0][6]=a22;
    res[7] = a5;  // if (res[0]!=0) res[0][7]=a5;
    a9=(a10*a19);
    a9=(a9-a11);
    res[8] = a9; // if (res[0]!=0) res[0][8]=a9;
    a10=(a10*a7);
    a10=(a10-a15);
    res[9] = a10; // if (res[0]!=0) res[0][9]=a10;
    a15=1.4503002514780097e-10;
    a15=(a15-a4);
    res[10] = a15; // if (res[0]!=0) res[0][10]=a15;
    res[11] = a5;  // if (res[0]!=0) res[0][11]=a5;
    a5=2.0000000000000000e-02;
    a23=(a5*a23);
    a4=8.9999999999999997e-02;
    a9=(a4*a9);
    a23=(a23+a9);
    a9=-4.2499999999999999e-01;
    a17=(a9*a17);
    a11=1.6250000000000001e-01;
    a19=(a11*a19);
    a20=2.0000000000000001e-01;
    a18=(a20*a2);
    a16=-2.2000000000000000e-01;
    a14=(a16*a1);
    a18=(a18-a14);
    a14 = arg[0]; // a14=arg[0]? arg[0][0] : 0;
    a18=(a18+a14);
    a19=(a19+a18);
    a17=(a17+a19);
    a23=(a23+a17);
    res[12] = a23; // if (res[0]!=0) res[0][12]=a23;
    a8=(a5*a8);
    a10=(a4*a10);
    a8=(a8+a10);
    a21=(a9*a21);
    a11=(a11*a7);
    a20=(a20*a1);
    a16=(a16*a2);
    a20=(a20+a16);
    a16 = arg[1]; // a16=arg[0]? arg[0][1] : 0;
    a20=(a20+a16);
    a11=(a11+a20);
    a21=(a21+a11);
    a8=(a8+a21);
    res[13] = a8; // if (res[0]!=0) res[0][13]=a8;
    a5=(a5*a22);
    a4=(a4*a15);
    a5=(a5+a4);
    a9=(a9*a0);
    a0=1.1180951480571861e+00;
    a9=(a9+a0);
    a5=(a5+a9);
    res[14] = a5; // if (res[0]!=0) res[0][14]=a5;
    a5=1.;
    res[15] = a5; // if (res[0]!=0) res[0][15]=a5;
    return 0;
}
int FG_eval::casadi_gripper_sphere(ADvector arg, ADvector& res){
    /* T_fk:(i0[9])->(o0[4x4]) */
    //read here arg.size():9
    //['base_y_base_x', 'base_theta_base_y', 'base_link_base_theta', 'right_arm_shoulder_pan_joint', 'right_arm_shoulder_lift_joint', 'right_arm_elbow_joint', 'right_arm_wrist_1_joint', 'right_arm_wrist_2_joint', 'right_arm_wrist_3_joint']
    AD<double> a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a4, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a5, a50, a51, a52, a53, a6, a7, a8, a9;
    a0=3.7493994566546440e-33;
    a1=1.2246467991473532e-16;
    a2 = arg[2]; // a2=arg[0]? arg[0][2] : 0;
    a3=cos(a2);
    a4=6.1232339957367660e-17;
    a5=-7.0710678118654746e-01;
    a2=sin(a2);
    a6=(a5*a2);
    a7=(a4*a6);
    a7=(a3-a7);
    a8=-6.1232339957367660e-17;
    a9=7.0710678118654757e-01;
    a10=(a9*a2);
    a11=(a8*a10);
    a7=(a7-a11);
    a11=(a1*a7);
    a12=(a4*a3);
    a12=(a12-a10);
    a11=(a11-a12);
    a13 = arg[3]; // a13=arg[0]? arg[0][3] : 0;
    a14=cos(a13);
    a15=(a11*a14);
    a16=-1.2246467991473532e-16;
    a12=(a16*a12);
    a12=(a12-a7);
    a13=sin(a13);
    a7=(a12*a13);
    a15=(a15+a7);
    a7 = arg[4]; // a7=arg[0]? arg[0][4] : 0;
    a17=cos(a7);
    a18=(a15*a17);
    a12=(a12*a14);
    a11=(a11*a13);
    a12=(a12-a11);
    a11=-2.0510342851533115e-10;
    a7=sin(a7);
    a19=(a11*a7);
    a20=(a12*a19);
    a18=(a18+a20);
    a20=(a8*a3);
    a20=(a20-a6);
    a10=(a0*a10);
    a20=(a20-a10);
    a10=(a20*a7);
    a18=(a18+a10);
    a10 = arg[5]; // a10=arg[0]? arg[0][5] : 0;
    a6=cos(a10);
    a21=(a18*a6);
    a22=(a11*a17);
    a23=(a12*a22);
    a15=(a15*a7);
    a23=(a23-a15);
    a15=(a20*a17);
    a23=(a23+a15);
    a10=sin(a10);
    a15=(a23*a10);
    a21=(a21+a15);
    a15 = arg[6]; // a15=arg[0]? arg[0][6] : 0;
    a24=cos(a15);
    a25=(a21*a24);
    a23=(a23*a6);
    a26=(a18*a10);
    a23=(a23-a26);
    a15=sin(a15);
    a26=(a23*a15);
    a25=(a25+a26);
    a26 = arg[7]; // a26=arg[0]? arg[0][7] : 0;
    a27=cos(a26);
    a28=(a25*a27);
    a23=(a23*a24);
    a29=(a21*a15);
    a23=(a23-a29);
    a26=sin(a26);
    a29=(a11*a26);
    a30=(a23*a29);
    a28=(a28+a30);
    a30=(a11*a20);
    a30=(a30-a12);
    a12=(a30*a26);
    a28=(a28+a12);
    a12 = arg[8]; // a12=arg[0]? arg[0][8] : 0;
    a31=cos(a12);
    a32=-1.2246467993985327e-16;
    a12=sin(a12);
    a33=(a32*a12);
    a33=(a31+a33);
    a34=(a28*a33);
    a35=(a11*a27);
    a36=(a23*a35);
    a25=(a25*a26);
    a36=(a36-a25);
    a25=(a30*a27);
    a36=(a36+a25);
    a25=(a16*a31);
    a37=-2.0510355098001109e-10;
    a38=(a37*a12);
    a25=(a25+a38);
    a38=(a36*a25);
    a34=(a34+a38);
    a38=(a11*a30);
    a38=(a38-a23);
    a39=(a16*a31);
    a39=(a39-a12);
    a40=(a38*a39);
    a34=(a34+a40);
    a40=(a0*a34);
    a32=(a32*a31);
    a32=(a32-a12);
    a41=(a28*a32);
    a42=(a37*a31);
    a43=(a16*a12);
    a42=(a42-a43);
    a43=(a36*a42);
    a41=(a41+a43);
    a12=(a16*a12);
    a12=(a12+a31);
    a31=(a38*a12);
    a41=(a41-a31);
    a31=(a8*a41);
    a40=(a40+a31);
    a31=1.2246467988961737e-16;
    a28=(a31*a28);
    a28=(a28+a36);
    a43=(a37*a38);
    a28=(a28+a43);
    a40=(a40+a28);
    res[0] = a40; // if (res[0]!=0) res[0][0]=a40;
    a5=(a5*a3);
    a43=(a4*a5);
    a43=(a2+a43);
    a9=(a9*a3);
    a44=(a8*a9);
    a43=(a43+a44);
    a1=(a1*a43);
    a44=(a4*a2);
    a44=(a44+a9);
    a1=(a1-a44);
    a45=(a1*a14);
    a16=(a16*a44);
    a16=(a16-a43);
    a43=(a16*a13);
    a45=(a45+a43);
    a43=(a45*a17);
    a16=(a16*a14);
    a1=(a1*a13);
    a16=(a16-a1);
    a1=(a16*a19);
    a43=(a43+a1);
    a1=(a8*a2);
    a1=(a1+a5);
    a9=(a0*a9);
    a1=(a1+a9);
    a9=(a1*a7);
    a43=(a43+a9);
    a9=(a43*a6);
    a5=(a16*a22);
    a45=(a45*a7);
    a5=(a5-a45);
    a45=(a1*a17);
    a5=(a5+a45);
    a45=(a5*a10);
    a9=(a9+a45);
    a45=(a9*a24);
    a5=(a5*a6);
    a44=(a43*a10);
    a5=(a5-a44);
    a44=(a5*a15);
    a45=(a45+a44);
    a44=(a45*a27);
    a5=(a5*a24);
    a46=(a9*a15);
    a5=(a5-a46);
    a46=(a5*a29);
    a44=(a44+a46);
    a46=(a11*a1);
    a46=(a46-a16);
    a16=(a46*a26);
    a44=(a44+a16);
    a16=(a44*a33);
    a47=(a5*a35);
    a45=(a45*a26);
    a47=(a47-a45);
    a45=(a46*a27);
    a47=(a47+a45);
    a45=(a47*a25);
    a16=(a16+a45);
    a45=(a11*a46);
    a45=(a45-a5);
    a48=(a45*a39);
    a16=(a16+a48);
    a48=(a0*a16);
    a49=(a44*a32);
    a50=(a47*a42);
    a49=(a49+a50);
    a50=(a45*a12);
    a49=(a49-a50);
    a50=(a8*a49);
    a48=(a48+a50);
    a44=(a31*a44);
    a44=(a44+a47);
    a50=(a37*a45);
    a44=(a44+a50);
    a48=(a48+a44);
    res[1] = a48; // if (res[0]!=0) res[0][1]=a48;
    a50=7.0710678118654746e-01;
    a51=(a50*a14);
    a52=8.6595605623549329e-17;
    a53=(a52*a13);
    a51=(a51+a53);
    a53=(a51*a17);
    a52=(a52*a14);
    a50=(a50*a13);
    a52=(a52-a50);
    a19=(a52*a19);
    a53=(a53+a19);
    a19=-7.0710678118654757e-01;
    a50=(a19*a7);
    a53=(a53+a50);
    a50=(a53*a6);
    a22=(a52*a22);
    a51=(a51*a7);
    a22=(a22-a51);
    a19=(a19*a17);
    a22=(a22+a19);
    a19=(a22*a10);
    a50=(a50+a19);
    a19=(a50*a24);
    a22=(a22*a6);
    a10=(a53*a10);
    a22=(a22-a10);
    a10=(a22*a15);
    a19=(a19+a10);
    a10=(a19*a27);
    a22=(a22*a24);
    a15=(a50*a15);
    a22=(a22-a15);
    a29=(a22*a29);
    a10=(a10+a29);
    a29=1.4503002514780097e-10;
    a29=(a29-a52);
    a52=(a29*a26);
    a10=(a10+a52);
    a33=(a10*a33);
    a35=(a22*a35);
    a19=(a19*a26);
    a35=(a35-a19);
    a27=(a29*a27);
    a35=(a35+a27);
    a25=(a35*a25);
    a33=(a33+a25);
    a11=(a11*a29);
    a11=(a11-a22);
    a39=(a11*a39);
    a33=(a33+a39);
    a0=(a0*a33);
    a32=(a10*a32);
    a42=(a35*a42);
    a32=(a32+a42);
    a12=(a11*a12);
    a32=(a32-a12);
    a12=(a8*a32);
    a0=(a0+a12);
    a31=(a31*a10);
    a31=(a31+a35);
    a37=(a37*a11);
    a31=(a31+a37);
    a0=(a0+a31);
    res[2] = a0; // if (res[0]!=0) res[0][2]=a0;
    a37=0.;
    res[3] = a37; // if (res[0]!=0) res[0][3]=a37;
    a10=(a4*a41);
    a10=(a34+a10);
    res[4] = a10; // if (res[0]!=0) res[0][4]=a10;
    a10=(a4*a49);
    a10=(a16+a10);
    res[5] = a10; // if (res[0]!=0) res[0][5]=a10;
    a10=(a4*a32);
    a10=(a33+a10);
    res[6] = a10; // if (res[0]!=0) res[0][6]=a10;
    res[7] = a37; // if (res[0]!=0) res[0][7]=a37;
    a34=(a8*a34);
    a34=(a34+a41);
    a28=(a4*a28);
    a34=(a34+a28);
    res[8] = a34; // if (res[0]!=0) res[0][8]=a34;
    a16=(a8*a16);
    a16=(a16+a49);
    a44=(a4*a44);
    a16=(a16+a44);
    res[9] = a16; // if (res[0]!=0) res[0][9]=a16;
    a8=(a8*a33);
    a8=(a8+a32);
    a4=(a4*a31);
    a8=(a8+a4);
    res[10] = a8; // if (res[0]!=0) res[0][10]=a8;
    res[11] = a37; // if (res[0]!=0) res[0][11]=a37;
    a37=1.2000000000000000e-01;
    a40=(a37*a40);
    a8=9.9599999999999994e-02;
    a36=(a8*a36);
    a4=-2.0428301480126979e-11;
    a38=(a4*a38);
    a36=(a36+a38);
    a38=-9.9699999999999997e-02;
    a23=(a38*a23);
    a31=-2.0448811822978519e-11;
    a32=(a31*a30);
    a23=(a23+a32);
    a32=-3.9219999999999999e-01;
    a21=(a32*a21);
    a33=1.3330000000000000e-01;
    a30=(a33*a30);
    a21=(a21+a30);
    a30=-4.2499999999999999e-01;
    a18=(a30*a18);
    a16=1.6250000000000001e-01;
    a20=(a16*a20);
    a44=2.0000000000000001e-01;
    a49=(a44*a3);
    a34=-2.2000000000000000e-01;
    a28=(a34*a2);
    a49=(a49-a28);
    a28 = arg[0]; // a28=arg[0]? arg[0][0] : 0;
    a49=(a49+a28);
    a20=(a20+a49);
    a18=(a18+a20);
    a21=(a21+a18);
    a23=(a23+a21);
    a36=(a36+a23);
    a40=(a40+a36);
    res[12] = a40; // if (res[0]!=0) res[0][12]=a40;
    a48=(a37*a48);
    a47=(a8*a47);
    a45=(a4*a45);
    a47=(a47+a45);
    a5=(a38*a5);
    a45=(a31*a46);
    a5=(a5+a45);
    a9=(a32*a9);
    a46=(a33*a46);
    a9=(a9+a46);
    a43=(a30*a43);
    a16=(a16*a1);
    a44=(a44*a2);
    a34=(a34*a3);
    a44=(a44+a34);
    a34 = arg[1]; // a34=arg[0]? arg[0][1] : 0;
    a44=(a44+a34);
    a16=(a16+a44);
    a43=(a43+a16);
    a9=(a9+a43);
    a5=(a5+a9);
    a47=(a47+a5);
    a48=(a48+a47);
    res[13] = a48; // if (res[0]!=0) res[0][13]=a48;
    a37=(a37*a0);
    a8=(a8*a35);
    a4=(a4*a11);
    a8=(a8+a4);
    a38=(a38*a22);
    a31=(a31*a29);
    a38=(a38+a31);
    a32=(a32*a50);
    a33=(a33*a29);
    a32=(a32+a33);
    a30=(a30*a53);
    a53=1.1180951480571861e+00;
    a30=(a30+a53);
    a32=(a32+a30);
    a38=(a38+a32);
    a8=(a8+a38);
    a37=(a37+a8);
    res[14] = a37; // if (res[0]!=0) res[0][14]=a37;
    a37=1.;
    res[15] = a37; // if (res[0]!=0) res[0][15]=a37;
    return 0;
}
int FG_eval::casadi_tool_pose(ADvector arg, ADvector& res){
    /* T_fk:(i0[9])->(o0[4x4]) */
    //read arg.size() = 9
    //同上
    AD<double> a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a4, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a5, a50, a51, a52, a53, a54, a55, a6, a7, a8, a9;
    a0=6.1232339957367660e-17;
    a1=3.7493994566546440e-33;
    a2=1.2246467991473532e-16;
    a3 = arg[2]; // a3=arg[0]? arg[0][2] : 0;
    a4=cos(a3);
    a5=-7.0710678118654746e-01;
    a3=sin(a3);
    a6=(a5*a3);
    a7=(a0*a6);
    a7=(a4-a7);
    a8=-6.1232339957367660e-17;
    a9=7.0710678118654757e-01;
    a10=(a9*a3);
    a11=(a8*a10);
    a7=(a7-a11);
    a11=(a2*a7);
    a12=(a0*a4);
    a12=(a12-a10);
    a11=(a11-a12);
    a13 = arg[3]; // a13=arg[0]? arg[0][3] : 0;
    a14=cos(a13);
    a15=(a11*a14);
    a16=-1.2246467991473532e-16;
    a12=(a16*a12);
    a12=(a12-a7);
    a13=sin(a13);
    a7=(a12*a13);
    a15=(a15+a7);
    a7 = arg[4]; // a7=arg[0]? arg[0][4] : 0;
    a17=cos(a7);
    a18=(a15*a17);
    a12=(a12*a14);
    a11=(a11*a13);
    a12=(a12-a11);
    a11=-2.0510342851533115e-10;
    a7=sin(a7);
    a19=(a11*a7);
    a20=(a12*a19);
    a18=(a18+a20);
    a20=(a8*a4);
    a20=(a20-a6);
    a10=(a1*a10);
    a20=(a20-a10);
    a10=(a20*a7);
    a18=(a18+a10);
    a10 = arg[5]; // a10=arg[0]? arg[0][5] : 0;
    a6=cos(a10);
    a21=(a18*a6);
    a22=(a11*a17);
    a23=(a12*a22);
    a15=(a15*a7);
    a23=(a23-a15);
    a15=(a20*a17);
    a23=(a23+a15);
    a10=sin(a10);
    a15=(a23*a10);
    a21=(a21+a15);
    a15 = arg[6]; // a15=arg[0]? arg[0][6] : 0;
    a24=cos(a15);
    a25=(a21*a24);
    a23=(a23*a6);
    a26=(a18*a10);
    a23=(a23-a26);
    a15=sin(a15);
    a26=(a23*a15);
    a25=(a25+a26);
    a26 = arg[7]; // a26=arg[0]? arg[0][7] : 0;
    a27=cos(a26);
    a28=(a25*a27);
    a23=(a23*a24);
    a29=(a21*a15);
    a23=(a23-a29);
    a26=sin(a26);
    a29=(a11*a26);
    a30=(a23*a29);
    a28=(a28+a30);
    a30=(a11*a20);
    a30=(a30-a12);
    a12=(a30*a26);
    a28=(a28+a12);
    a12 = arg[8]; // a12=arg[0]? arg[0][8] : 0;
    a31=cos(a12);
    a32=-1.2246467993985327e-16;
    a12=sin(a12);
    a33=(a32*a12);
    a33=(a31+a33);
    a34=(a28*a33);
    a35=(a11*a27);
    a36=(a23*a35);
    a25=(a25*a26);
    a36=(a36-a25);
    a25=(a30*a27);
    a36=(a36+a25);
    a25=(a16*a31);
    a37=-2.0510355098001109e-10;
    a38=(a37*a12);
    a25=(a25+a38);
    a38=(a36*a25);
    a34=(a34+a38);
    a38=(a11*a30);
    a38=(a38-a23);
    a39=(a16*a31);
    a39=(a39-a12);
    a40=(a38*a39);
    a34=(a34+a40);
    a40=(a1*a34);
    a32=(a32*a31);
    a32=(a32-a12);
    a41=(a28*a32);
    a42=(a37*a31);
    a43=(a16*a12);
    a42=(a42-a43);
    a43=(a36*a42);
    a41=(a41+a43);
    a12=(a16*a12);
    a12=(a12+a31);
    a31=(a38*a12);
    a41=(a41-a31);
    a31=(a8*a41);
    a40=(a40+a31);
    a31=1.2246467988961737e-16;
    a28=(a31*a28);
    a28=(a28+a36);
    a43=(a37*a38);
    a28=(a28+a43);
    a40=(a40+a28);
    a43=(a0*a40);
    a44=(a0*a41);
    a44=(a34+a44);
    a43=(a43+a44);
    res[0] = a43; // if (res[0]!=0) res[0][0]=a43;
    a5=(a5*a4);
    a43=(a0*a5);
    a43=(a3+a43);
    a9=(a9*a4);
    a45=(a8*a9);
    a43=(a43+a45);
    a2=(a2*a43);
    a45=(a0*a3);
    a45=(a45+a9);
    a2=(a2-a45);
    a46=(a2*a14);
    a16=(a16*a45);
    a16=(a16-a43);
    a43=(a16*a13);
    a46=(a46+a43);
    a43=(a46*a17);
    a16=(a16*a14);
    a2=(a2*a13);
    a16=(a16-a2);
    a2=(a16*a19);
    a43=(a43+a2);
    a2=(a8*a3);
    a2=(a2+a5);
    a9=(a1*a9);
    a2=(a2+a9);
    a9=(a2*a7);
    a43=(a43+a9);
    a9=(a43*a6);
    a5=(a16*a22);
    a46=(a46*a7);
    a5=(a5-a46);
    a46=(a2*a17);
    a5=(a5+a46);
    a46=(a5*a10);
    a9=(a9+a46);
    a46=(a9*a24);
    a5=(a5*a6);
    a45=(a43*a10);
    a5=(a5-a45);
    a45=(a5*a15);
    a46=(a46+a45);
    a45=(a46*a27);
    a5=(a5*a24);
    a47=(a9*a15);
    a5=(a5-a47);
    a47=(a5*a29);
    a45=(a45+a47);
    a47=(a11*a2);
    a47=(a47-a16);
    a16=(a47*a26);
    a45=(a45+a16);
    a16=(a45*a33);
    a48=(a5*a35);
    a46=(a46*a26);
    a48=(a48-a46);
    a46=(a47*a27);
    a48=(a48+a46);
    a46=(a48*a25);
    a16=(a16+a46);
    a46=(a11*a47);
    a46=(a46-a5);
    a49=(a46*a39);
    a16=(a16+a49);
    a49=(a1*a16);
    a50=(a45*a32);
    a51=(a48*a42);
    a50=(a50+a51);
    a51=(a46*a12);
    a50=(a50-a51);
    a51=(a8*a50);
    a49=(a49+a51);
    a45=(a31*a45);
    a45=(a45+a48);
    a51=(a37*a46);
    a45=(a45+a51);
    a49=(a49+a45);
    a51=(a0*a49);
    a52=(a0*a50);
    a52=(a16+a52);
    a51=(a51+a52);
    res[1] = a51; // if (res[0]!=0) res[0][1]=a51;
    a51=7.0710678118654746e-01;
    a53=(a51*a14);
    a54=8.6595605623549329e-17;
    a55=(a54*a13);
    a53=(a53+a55);
    a55=(a53*a17);
    a54=(a54*a14);
    a51=(a51*a13);
    a54=(a54-a51);
    a19=(a54*a19);
    a55=(a55+a19);
    a19=-7.0710678118654757e-01;
    a51=(a19*a7);
    a55=(a55+a51);
    a51=(a55*a6);
    a22=(a54*a22);
    a53=(a53*a7);
    a22=(a22-a53);
    a19=(a19*a17);
    a22=(a22+a19);
    a19=(a22*a10);
    a51=(a51+a19);
    a19=(a51*a24);
    a22=(a22*a6);
    a10=(a55*a10);
    a22=(a22-a10);
    a10=(a22*a15);
    a19=(a19+a10);
    a10=(a19*a27);
    a22=(a22*a24);
    a15=(a51*a15);
    a22=(a22-a15);
    a29=(a22*a29);
    a10=(a10+a29);
    a29=1.4503002514780097e-10;
    a29=(a29-a54);
    a54=(a29*a26);
    a10=(a10+a54);
    a33=(a10*a33);
    a35=(a22*a35);
    a19=(a19*a26);
    a35=(a35-a19);
    a27=(a29*a27);
    a35=(a35+a27);
    a25=(a35*a25);
    a33=(a33+a25);
    a11=(a11*a29);
    a11=(a11-a22);
    a39=(a11*a39);
    a33=(a33+a39);
    a39=(a1*a33);
    a32=(a10*a32);
    a42=(a35*a42);
    a32=(a32+a42);
    a12=(a11*a12);
    a32=(a32-a12);
    a12=(a8*a32);
    a39=(a39+a12);
    a31=(a31*a10);
    a31=(a31+a35);
    a37=(a37*a11);
    a31=(a31+a37);
    a39=(a39+a31);
    a37=(a0*a39);
    a10=(a0*a32);
    a10=(a33+a10);
    a37=(a37+a10);
    res[2] = a37; // if (res[0]!=0) res[0][2]=a37;
    a37=0.;
    res[3] = a37; // if (res[0]!=0) res[0][3]=a37;
    a12=(a8*a40);
    a42=(a1*a44);
    a12=(a12+a42);
    a34=(a8*a34);
    a34=(a34+a41);
    a28=(a0*a28);
    a34=(a34+a28);
    a12=(a12+a34);
    res[4] = a12; // if (res[0]!=0) res[0][4]=a12;
    a12=(a8*a49);
    a28=(a1*a52);
    a12=(a12+a28);
    a16=(a8*a16);
    a16=(a16+a50);
    a45=(a0*a45);
    a16=(a16+a45);
    a12=(a12+a16);
    res[5] = a12; // if (res[0]!=0) res[0][5]=a12;
    a12=(a8*a39);
    a1=(a1*a10);
    a12=(a12+a1);
    a33=(a8*a33);
    a33=(a33+a32);
    a31=(a0*a31);
    a33=(a33+a31);
    a12=(a12+a33);
    res[6] = a12; // if (res[0]!=0) res[0][6]=a12;
    res[7] = a37; // if (res[0]!=0) res[0][7]=a37;
    a44=(a8*a44);
    a40=(a40+a44);
    a34=(a0*a34);
    a40=(a40+a34);
    res[8] = a40; // if (res[0]!=0) res[0][8]=a40;
    a52=(a8*a52);
    a49=(a49+a52);
    a16=(a0*a16);
    a49=(a49+a16);
    res[9] = a49; // if (res[0]!=0) res[0][9]=a49;
    a8=(a8*a10);
    a39=(a39+a8);
    a0=(a0*a33);
    a39=(a39+a0);
    res[10] = a39; // if (res[0]!=0) res[0][10]=a39;
    res[11] = a37; // if (res[0]!=0) res[0][11]=a37;
    a37=9.9599999999999994e-02;
    a36=(a37*a36);
    a39=-2.0428301480126979e-11;
    a38=(a39*a38);
    a36=(a36+a38);
    a38=-9.9699999999999997e-02;
    a23=(a38*a23);
    a0=-2.0448811822978519e-11;
    a33=(a0*a30);
    a23=(a23+a33);
    a33=-3.9219999999999999e-01;
    a21=(a33*a21);
    a8=1.3330000000000000e-01;
    a30=(a8*a30);
    a21=(a21+a30);
    a30=-4.2499999999999999e-01;
    a18=(a30*a18);
    a10=1.6250000000000001e-01;
    a20=(a10*a20);
    a49=2.0000000000000001e-01;
    a16=(a49*a4);
    a52=-2.2000000000000000e-01;
    a40=(a52*a3);
    a16=(a16-a40);
    a40 = arg[0]; // a40=arg[0]? arg[0][0] : 0;
    a16=(a16+a40);
    a20=(a20+a16);
    a18=(a18+a20);
    a21=(a21+a18);
    a23=(a23+a21);
    a36=(a36+a23);
    res[12] = a36; // if (res[0]!=0) res[0][12]=a36;
    a48=(a37*a48);
    a46=(a39*a46);
    a48=(a48+a46);
    a5=(a38*a5);
    a46=(a0*a47);
    a5=(a5+a46);
    a9=(a33*a9);
    a47=(a8*a47);
    a9=(a9+a47);
    a43=(a30*a43);
    a10=(a10*a2);
    a49=(a49*a3);
    a52=(a52*a4);
    a49=(a49+a52);
    a52 = arg[1]; // a52=arg[0]? arg[0][1] : 0;
    a49=(a49+a52);
    a10=(a10+a49);
    a43=(a43+a10);
    a9=(a9+a43);
    a5=(a5+a9);
    a48=(a48+a5);
    res[13] = a48; // if (res[0]!=0) res[0][13]=a48;
    a37=(a37*a35);
    a39=(a39*a11);
    a37=(a37+a39);
    a38=(a38*a22);
    a0=(a0*a29);
    a38=(a38+a0);
    a33=(a33*a51);
    a8=(a8*a29);
    a33=(a33+a8);
    a30=(a30*a55);
    a55=1.1180951480571861e+00;
    a30=(a30+a55);
    a33=(a33+a30);
    a38=(a38+a33);
    a37=(a37+a38);
    res[14] = a37; // if (res[0]!=0) res[0][14]=a37;
    a37=1.;
    res[15] = a37; // if (res[0]!=0) res[0][15]=a37;
    return 0;
}
int FG_eval::casadi_base_lf(ADvector arg, ADvector& res){
    /* T_fk:(i0[3])->(o0[4x4]) */
    //read arg.size():3
    //['base_y_base_x', 'base_theta_base_y', 'base_link_base_theta']
    AD<double> a0, a1, a2, a3, a4, a5, a6;
    a0 = arg[2]; // a0=arg[0]? arg[0][2] : 0;
    a1=cos(a0);
    res[0] = a1; // if (res[0]!=0) res[0][0]=a1;
    a0=sin(a0);
    res[1] = a0; // if (res[0]!=0) res[0][1]=a0;
    a2=0.;
    res[2] = a2; // if (res[0]!=0) res[0][2]=a2;
    res[3] = a2; // if (res[0]!=0) res[0][3]=a2;
    a3=(-a0);
    res[4] = a3; // if (res[0]!=0) res[0][4]=a3;
    res[5] = a1; // if (res[0]!=0) res[0][5]=a1;
    res[6] = a2; // if (res[0]!=0) res[0][6]=a2;
    res[7] = a2; // if (res[0]!=0) res[0][7]=a2;
    res[8] = a2; // if (res[0]!=0) res[0][8]=a2;
    res[9] = a2; // if (res[0]!=0) res[0][9]=a2;
    a3=1.;
    res[10] = a3; // if (res[0]!=0) res[0][10]=a3;
    res[11] = a2; // if (res[0]!=0) res[0][11]=a2;
    a2=2.0000000000000001e-01;
    a4=(a2*a1);
    a5=1.4999999999999999e-01;
    a6=(a5*a0);
    a4=(a4-a6);
    a6 = arg[0]; // a6=arg[0]? arg[0][0] : 0;
    a4=(a4+a6);
    res[12] = a4; // if (res[0]!=0) res[0][12]=a4;
    a2=(a2*a0);
    a5=(a5*a1);
    a2=(a2+a5);
    a5 = arg[1]; // a5=arg[0]? arg[0][1] : 0;
    a2=(a2+a5);
    res[13] = a2; // if (res[0]!=0) res[0][13]=a2;
    a2=1.6965000000000000e-01;
    res[14] = a2; // if (res[0]!=0) res[0][14]=a2;
    res[15] = a3; // if (res[0]!=0) res[0][15]=a3;
    return 0;
}
int FG_eval::casadi_base_rf(ADvector arg, ADvector& res){
    /* T_fk:(i0[3])->(o0[4x4]) */
    //read arg.size():3
    //['base_y_base_x', 'base_theta_base_y', 'base_link_base_theta']
    AD<double> a0, a1, a2, a3, a4, a5, a6;
    a0 = arg[2]; // a0=arg[0]? arg[0][2] : 0;
    a1=cos(a0);
    res[0] = a1; // if (res[0]!=0) res[0][0]=a1;
    a0=sin(a0);
    res[1] = a0; // if (res[0]!=0) res[0][1]=a0;
    a2=0.;
    res[2] = a2; // if (res[0]!=0) res[0][2]=a2;
    res[3] = a2; // if (res[0]!=0) res[0][3]=a2;
    a3=(-a0);
    res[4] = a3; // if (res[0]!=0) res[0][4]=a3;
    res[5] = a1; // if (res[0]!=0) res[0][5]=a1;
    res[6] = a2; // if (res[0]!=0) res[0][6]=a2;
    res[7] = a2; // if (res[0]!=0) res[0][7]=a2;
    res[8] = a2; // if (res[0]!=0) res[0][8]=a2;
    res[9] = a2; // if (res[0]!=0) res[0][9]=a2;
    a3=1.;
    res[10] = a3; // if (res[0]!=0) res[0][10]=a3;
    res[11] = a2; // if (res[0]!=0) res[0][11]=a2;
    a2=2.0000000000000001e-01;
    a4=(a2*a1);
    a5=-1.4999999999999999e-01;
    a6=(a5*a0);
    a4=(a4-a6);
    a6 = arg[0]; // a6=arg[0]? arg[0][0] : 0;
    a4=(a4+a6);
    res[12] = a4; // if (res[0]!=0) res[0][12]=a4;
    a2=(a2*a0);
    a5=(a5*a1);
    a2=(a2+a5);
    a5 = arg[1]; // a5=arg[0]? arg[0][1] : 0;
    a2=(a2+a5);
    res[13] = a2; // if (res[0]!=0) res[0][13]=a2;
    a2=1.6965000000000000e-01;
    res[14] = a2; // if (res[0]!=0) res[0][14]=a2;
    res[15] = a3; // if (res[0]!=0) res[0][15]=a3;
    return 0;
}
int FG_eval::casadi_base_lr(ADvector arg, ADvector& res)
{
    /* T_fk:(i0[3])->(o0[4x4]) */
    //read arg.size():3
    //['base_y_base_x', 'base_theta_base_y', 'base_link_base_theta']
    AD<double> a0, a1, a2, a3, a4, a5, a6;
    a0 = arg[2]; // a0=arg[0]? arg[0][2] : 0;
    a1=cos(a0);
    res[0] = a1; // if (res[0]!=0) res[0][0]=a1;
    a0=sin(a0);
    res[1] = a0; // if (res[0]!=0) res[0][1]=a0;
    a2=0.;
    res[2] = a2; // if (res[0]!=0) res[0][2]=a2;
    res[3] = a2; // if (res[0]!=0) res[0][3]=a2;
    a3=(-a0);
    res[4] = a3; // if (res[0]!=0) res[0][4]=a3;
    res[5] = a1; // if (res[0]!=0) res[0][5]=a1;
    res[6] = a2; // if (res[0]!=0) res[0][6]=a2;
    res[7] = a2; // if (res[0]!=0) res[0][7]=a2;
    res[8] = a2; // if (res[0]!=0) res[0][8]=a2;
    res[9] = a2; // if (res[0]!=0) res[0][9]=a2;
    a3=1.;
    res[10] = a3; // if (res[0]!=0) res[0][10]=a3;
    res[11] = a2; // if (res[0]!=0) res[0][11]=a2;
    a2=-2.0000000000000001e-01;
    a4=(a2*a1);
    a5=1.4999999999999999e-01;
    a6=(a5*a0);
    a4=(a4-a6);
    a6 = arg[0]; // a6=arg[0]? arg[0][0] : 0;
    a4=(a4+a6);
    res[12] = a4; // if (res[0]!=0) res[0][12]=a4;
    a2=(a2*a0);
    a5=(a5*a1);
    a2=(a2+a5);
    a5 = arg[1]; // a5=arg[0]? arg[0][1] : 0;
    a2=(a2+a5);
    res[13] = a2; // if (res[0]!=0) res[0][13]=a2;
    a2=1.6965000000000000e-01;
    res[14] = a2; // if (res[0]!=0) res[0][14]=a2;
    res[15] = a3; // if (res[0]!=0) res[0][15]=a3;
    return 0;
}
int FG_eval::casadi_base_rr(ADvector arg, ADvector& res)
{
    /* T_fk:(i0[3])->(o0[4x4]) */
    //read arg.size():3
    //['base_y_base_x', 'base_theta_base_y', 'base_link_base_theta']
    AD<double> a0, a1, a2, a3, a4, a5, a6;
    a0 = arg[2]; // a0=arg[0]? arg[0][2] : 0;
    a1=cos(a0);
    res[0] = a1; // if (res[0]!=0) res[0][0]=a1;
    a0=sin(a0);
    res[1] = a0; // if (res[0]!=0) res[0][1]=a0;
    a2=0.;
    res[2] = a2; // if (res[0]!=0) res[0][2]=a2;
    res[3] = a2; // if (res[0]!=0) res[0][3]=a2;
    a3=(-a0);
    res[4] = a3; // if (res[0]!=0) res[0][4]=a3;
    res[5] = a1; // if (res[0]!=0) res[0][5]=a1;
    res[6] = a2; // if (res[0]!=0) res[0][6]=a2;
    res[7] = a2; // if (res[0]!=0) res[0][7]=a2;
    res[8] = a2; // if (res[0]!=0) res[0][8]=a2;
    res[9] = a2; // if (res[0]!=0) res[0][9]=a2;
    a3=1.;
    res[10] = a3; // if (res[0]!=0) res[0][10]=a3;
    res[11] = a2; // if (res[0]!=0) res[0][11]=a2;
    a2=-2.0000000000000001e-01;
    a4=(a2*a1);
    a5=-1.4999999999999999e-01;
    a6=(a5*a0);
    a4=(a4-a6);
    a6 = arg[0]; // a6=arg[0]? arg[0][0] : 0;
    a4=(a4+a6);
    res[12] = a4; // if (res[0]!=0) res[0][12]=a4;
    a2=(a2*a0);
    a5=(a5*a1);
    a2=(a2+a5);
    a5 = arg[1]; // a5=arg[0]? arg[0][1] : 0;
    a2=(a2+a5);
    res[13] = a2; // if (res[0]!=0) res[0][13]=a2;
    a2=1.6965000000000000e-01;
    res[14] = a2; // if (res[0]!=0) res[0][14]=a2;
    res[15] = a3; // if (res[0]!=0) res[0][15]=a3;
    return 0;
}


// AD<double> FG_eval::calculate_terminalCost(tf::Vector3 ee_position, tf::Vector3 ee_orientation)
// {
//     //输入的是 预测的EE_tool的position和Orientation
//     AD<double> error_squared = 0.0;
//     error_squared += CppAD::pow(ee_position[0] - EE_X, 2);
//     error_squared += CppAD::pow(ee_position[1] - EE_Y, 2);
//     error_squared += CppAD::pow(ee_position[2] - EE_Z, 2);
//     error_squared += CppAD::pow(ee_orientation[0] - EE_ROLL, 2);
//     error_squared += CppAD::pow(ee_orientation[1] - EE_PITCH, 2);
//     error_squared += CppAD::pow(ee_orientation[2] - EE_YAW, 2);
//     return _w_hard_EE_tool * error_squared;
// }

void FG_eval::operator()(ADvector& fg, const ADvector& vars)
{
    std::cout << "进来几次啊" << std::endl;
    // first,fg[0] represent for Objective Function
    fg[0] = 0;
    cost_distx = 0;
    cost_disty = 0;
    cost_etheta = 0;


    cost_vx = 0;
    cost_vy = 0;
    cost_angvel = 0;

    for (int i = 0; i < _mpc_steps; i++)
    {
        ////std::cout << "here is got ? debug1" << std::endl;

        // todo z represent theta for traj_pub
        //!这不和正常的double 也可以做减法么
        ////std::cout << "here is got ? debug4" << std::endl;
        // for(int i = 0; i < _mpc_trackTraj.AnglesList.size(); i++)
        // {
        //     std::cout << "track traj angle_" << i << std::endl;
        //     std::cout << _mpc_trackTraj.AnglesList[i].base_x << "," << _mpc_trackTraj.AnglesList[i].base_y << "," << _mpc_trackTraj.AnglesList[i].base_theta
        //     << "," << _mpc_trackTraj.AnglesList[i].joint1 << ","  << _mpc_trackTraj.AnglesList[i].joint2 << "," << _mpc_trackTraj.AnglesList[i].joint3 << "," <<
        //     _mpc_trackTraj.AnglesList[i].joint4 << "," << _mpc_trackTraj.AnglesList[i].joint5 << "," << _mpc_trackTraj.AnglesList[i].joint6 << std::endl;
        // }
        //debug 这是位置误差
        fg[0] += _w_distx * CppAD::pow(vars[_x_start + i] - _mpc_trackTraj.AnglesList[i].base_x, 2); // x error

        cost_distx =  _w_distx * CppAD::pow(vars[_x_start + i] - _mpc_trackTraj.AnglesList[i].base_x, 2); // x error
        cout << "cost_distx: " << cost_distx << ",";

        fg[0] += _w_disty * CppAD::pow(vars[_y_start + i] - _mpc_trackTraj.AnglesList[i].base_y, 2); // y error

        cost_disty =  _w_disty * CppAD::pow(vars[_y_start + i] - _mpc_trackTraj.AnglesList[i].base_y, 2); // y error
        cout << "cost_disty: " << cost_disty << ",";
        fg[0] += _w_etheta * CppAD::pow(vars[_theta_start + i] - _mpc_trackTraj.AnglesList[i].base_theta, 2); // theta error

        cost_etheta =  _w_etheta * CppAD::pow(vars[_theta_start + i] - _mpc_trackTraj.AnglesList[i].base_theta, 2); // theta error
        cout << "cost_etheta: " << cost_etheta << ",";

        //debug 计算障碍物误差
        //debug 先不考虑避障
        Dvector pedestrian_predpos(3);
        pedestrian_predpos[0] = _init_sphere[0][0];
        pedestrian_predpos[1] = _init_sphere[0][1] + i * _dt * _pedestrian_vel;
        pedestrian_predpos[2] = _init_sphere[0][2];
        // std::cout << "pedestrian_predpos: " << pedestrian_predpos[0] << "," << pedestrian_predpos[1] <<
        //     "," << pedestrian_predpos[2] << std::endl;

        if(i == 0)
        {//第一个的目标函数值的计算，是由/tf的结果给出

            //read 计算和行人的距离，并惩罚
            //_init_spere[1] -> base_left_front_sphere
            AD<double> distance_lf_ = CppAD::sqrt(CppAD::pow(_init_sphere[1][0] - pedestrian_predpos[0], 2) + CppAD::pow(_init_sphere[1][1] - pedestrian_predpos[1], 2));
            fg[0] += _w_base_collision * barried_func_base_(distance_lf_);

            ////AD<double> distance_lb_ = CppAD::sqrt(CppAD::pow(_init_sphere[3][0] - pedestrian_predpos[0], 2) + CppAD::pow(_init_sphere[3][1] - pedestrian_predpos[1], 2));
            cout << "distance_lf_: " << distance_lf_ << ",";
            //_init_spere[2] -> base_right_front_sphere
            AD<double> distance_rf_ = CppAD::sqrt(CppAD::pow(_init_sphere[2][0] - pedestrian_predpos[0], 2) + CppAD::pow(_init_sphere[2][1] - pedestrian_predpos[1], 2));
            fg[0] += _w_base_collision * barried_func_base_(distance_rf_);
            ////AD<double> distance_rb_ = CppAD::sqrt(CppAD::pow(_init_sphere[4][0] - pedestrian_predpos[0], 2) + CppAD::pow(_init_sphere[4][1] - pedestrian_predpos[1], 2));
            cout << "distance_rf_: " << distance_rf_ << ",";
            //_init_spere[3] -> base_left_rear_sphere
            AD<double> distance_lr_ = CppAD::sqrt(CppAD::pow(_init_sphere[3][0] - pedestrian_predpos[0], 2) + CppAD::pow(_init_sphere[3][1] - pedestrian_predpos[1], 2));
            fg[0] += _w_base_collision * barried_func_base_(distance_lr_);
            //_init_spere[4] -> base_right_rear_sphere
            AD<double> distance_rr_ = CppAD::sqrt(CppAD::pow(_init_sphere[4][0] - pedestrian_predpos[0], 2) + CppAD::pow(_init_sphere[4][1] - pedestrian_predpos[1], 2));
            fg[0] += _w_base_collision * barried_func_base_(distance_rr_);

            cout << "cost_base_lf: " << barried_func_base_(distance_lf_) << "," <<
                "cost_base_rf: " << barried_func_base_(distance_rf_) << "," << std::endl;

        }
        else
        {
            //for(int j = 1; j < _mpc_steps; j++)
            //从第二个开始是预测的，需要自己计算
            //read 计算和行人的距离并惩罚
            ADvector arg_lf(3);
            arg_lf[0] = vars[_x_start + i];
            arg_lf[1] = vars[_y_start + i];
            arg_lf[2] = vars[_theta_start + i];
            ADvector res_lf_(16);
            casadi_base_lf(arg_lf, res_lf_);
            //res[12] -> x , res[13] -> y , res[14] -> z
            AD<double> distance_lf_ = CppAD::sqrt(CppAD::pow(res_lf_[12] - pedestrian_predpos[0], 2) + CppAD::pow(res_lf_[13] - pedestrian_predpos[1], 2));
            fg[0] += _w_base_collision * barried_func_base_(distance_lf_);

            ADvector arg_rf(3);
            arg_rf[0] = vars[_x_start + i];
            arg_rf[1] = vars[_y_start + i];
            arg_rf[2] = vars[_theta_start + i];
            ADvector res_rf_(16);
            casadi_base_rf(arg_rf, res_rf_);
            //res[12] -> x , res[13] -> y , res[14] -> z
            AD<double> distance_rf_ = CppAD::sqrt(CppAD::pow(res_rf_[12] - pedestrian_predpos[0], 2) + CppAD::pow(res_rf_[13] - pedestrian_predpos[1], 2));
            fg[0] += _w_base_collision * barried_func_base_(distance_rf_);

            ADvector arg_lr(3);
            arg_lr[0] = vars[_x_start + i];
            arg_lr[1] = vars[_y_start + i];
            arg_lr[2] = vars[_theta_start + i];
            ADvector res_lr_(16);
            casadi_base_lr(arg_lr, res_lr_);
            //res[12] -> x , res[13] -> y , res[14] -> z
            AD<double> distance_lr_ = CppAD::sqrt(CppAD::pow(res_lr_[12] - pedestrian_predpos[0], 2) + CppAD::pow(res_lr_[13] - pedestrian_predpos[1], 2));
            fg[0] += _w_base_collision * barried_func_base_(distance_lr_);

            ADvector arg_rr(3);
            arg_rr[0] = vars[_x_start + i];
            arg_rr[1] = vars[_y_start + i];
            arg_rr[2] = vars[_theta_start + i];
            ADvector res_rr_(16);
            casadi_base_rr(arg_rr, res_rr_);
            //res[12] -> x , res[13] -> y , res[14] -> z
            AD<double> distance_rr_ = CppAD::sqrt(CppAD::pow(res_rr_[12] - pedestrian_predpos[0], 2) + CppAD::pow(res_rr_[13] - pedestrian_predpos[1], 2));
            fg[0] += _w_base_collision * barried_func_base_(distance_rr_);

            //TODO 计算和电视柜子平面的距离，并惩罚防止撞到电视柜
            Eigen::Vector3d tv_normal; //电视柜的法向量
            Eigen::Vector3d tv_point; //电视柜表面的点
            //计算向量点乘，也就是对应相乘相加
            //maybe：
            auto dist_ = (tv_point[0] - res_lf_[0]) * tv_normal[0] +
                                (tv_point[1] - res_lf_[1]) * tv_normal[1] + (tv_point[2] - res_lf_[2]) * tv_normal[2];

            //TODO 计算到柜子正面、侧面的距离，并惩罚，防止撞到衣柜
            Eigen::Vector3d closet_normal; //衣柜的法向量
            Eigen::Vector3d closet_point; //衣柜表面的点
            //计算向量点乘，也就是对应相乘相加
            //maybe：
            auto dist_ = (closet_point[0] - res_lf_[0]) * closet_normal[0] +
                                (closet_point[1] - res_lf_[1]) * closet_normal[1] + (closet_point[2] - res_lf_[2]) * closet_normal[2];
            //还有一个

            //TODO 计算到移动柜门的距离
            //柜门打开的角度
            Eigen::Vector3d closet_door_normal; //柜门的法向量 //最初的，那就是相对于door系
            double door_angle = _mpc_trackTraj.AnglesList[i].joint_door;
            //旋转轴
            Eigen::Vector3d door_axis;
            //新的door系，相对于之前的旋转矩阵，这个是new_door系
            Eigen::AngleAxisd rotation_(door_angle, door_axis);

            Eigen::Matrix3d new_door = rotation_.toRotationMatrix();
            //new_door的法向量：
            Eigen::Vector3d new_door_normal = new_door * closet_door_normal;

            //todo
            //new_door系上的点（x,y,z） -> 旋转矩阵到 door系 -> 旋转矩阵到world系
        }

        // ADvector arg_tool_pose(9);
        // arg_tool_pose[0] = vars[_x_start + i];
        // arg_tool_pose[1] = vars[_y_start + i];
        // arg_tool_pose[2] = vars[_theta_start + i];
        // arg_tool_pose[3] = vars[_joint1_start + i];
        // arg_tool_pose[4] = vars[_joint2_start + i];
        // arg_tool_pose[5] = vars[_joint3_start + i];
        // arg_tool_pose[6] = vars[_joint4_start + i];
        // arg_tool_pose[7] = vars[_joint5_start + i];
        // arg_tool_pose[8] = vars[_joint6_start + i];
        // ADvector res_tool_pose_(16);
        // casadi_tool_pose(arg_tool_pose, res_tool_pose_);

        // cout << "ee_pos: " << res_tool_pose_[12] << "," << res_tool_pose_[13] << "," << res_tool_pose_[14] << endl;

        // fg[0] += _w_vector_terminal[i] * (CppAD::pow(res_tool_pose_[12] - EE_X, 2) + CppAD::pow(res_tool_pose_[13] - EE_Y, 2)
        //                                     + CppAD::pow(res_tool_pose_[14] - EE_Z, 2));

        // cost_etheta +=
        //         _w_vector_terminal[i] * (CppAD::pow(res_tool_pose_[12] - EE_X, 2) + CppAD::pow(res_tool_pose_[13] - EE_Y, 2)
        //                 + CppAD::pow(res_tool_pose_[14] - EE_Z, 2));
    }
    cout << "-----------------------------------------------" <<endl;
    //diff -> get the angle-acc & jerk
    for (int i = 0; i < _mpc_steps - 1; i++) {
        fg[0] += _w_vel * CppAD::pow(vars[_x_start + i + 1] - vars[_x_start + i], 2);

        fg[0] += _w_vel * CppAD::pow(vars[_y_start + i + 1] - vars[_y_start + i], 2);

        fg[0] += _w_angvel * CppAD::pow(vars[_theta_start + i + 1] - vars[_theta_start + i], 2);
    }

    //!Minimize the use of actuators.速度变化顺滑些，惩罚加速度 //0306 15:13实验证明，必须要有这一项，否则机器人抖得厉害
    //diff -> get the angle-acc & jerk
    for (int i = 0; i < _mpc_steps - 2; i++) {
        fg[0] += _w_acc * CppAD::pow(vars[_vx_start + i + 1] - vars[_vx_start + i], 2);
        fg[0] += _w_acc * CppAD::pow(vars[_vy_start + i + 1] - vars[_vy_start + i], 2);
        fg[0] += _w_angacc * CppAD::pow(vars[_angvel_start + i + 1] - vars[_angvel_start + i], 2);
    }

    // fg[x] for constraints
    //! 初始的约束：其实就是把自变量的值付过来
    // Initial constraints
    // 等式约束，为了规定初值
    //! 目标函数F，约束函数G
    ////std::cout << "F[1] ******************* DEBUG 1" << std::endl;

    fg[1 + _x_start] = vars[_x_start];
    fg[1 + _y_start] = vars[_y_start];
    fg[1 + _theta_start] = vars[_theta_start];

    ////std::cout << "F[1] ******************* DEBUG 2" << std::endl;
    //debug fg.size() 等于 ng+1 , ng是g(x)的数量,约束一共又两部分组成,纯x的不等式约束,和g(x)的等式&不等式约束

    //系统状态方程->视为等式约束
    for (int i = 0; i < _mpc_steps - 1; i++)
    {
        // The state at time t+1 .
        AD<double> x1 = vars[_x_start + i + 1];
        AD<double> y1 = vars[_y_start + i + 1];
        AD<double> theta1 = vars[_theta_start + i + 1];
        ////std::cout << "F[" << i << "] #################################### STATE 1" << std::endl;

        // The state at time t.
        AD<double> x0 = vars[_x_start + i];
        AD<double> y0 = vars[_y_start + i];
        AD<double> theta0 = vars[_theta_start + i];

        ////std::cout << "F[" << i << "] #################################### STATE 2" << std::endl;

        // Only consider the actuation at time t.仅考虑时间t时的输入（驱动）。
        AD<double> vx_ = vars[_vx_start + i];
        AD<double> vy_ = vars[_vy_start + i];
        AD<double> w_ = vars[_angvel_start + i];

        // std::cout << "速度： " << vx_ << ","
        //           << vy_ << ","
        //           << w_ << ","
        //           << std::endl;
        ////std::cout << "F[" << i << "] #################################### STATE 3" << std::endl;

        //! 使用运动学的积分方程,这些都是线性的，所以不需要什么运动学方程，直接乘时间就行。毕竟都是在world系下进行的
        // TODO: Setup the rest of the model constraints在这里将系统的状态方程填入
        fg[2 + _x_start + i] = x1 - (x0 + vx_ * _dt);
        fg[2 + _y_start + i] = y1 - (y0 + vy_ * _dt);
        fg[2 + _theta_start + i] = theta1 - (theta0 + w_ * _dt);

        ////std::cout << "F[" << i << "] #################################### STATE 4" << std::endl;
    }

}