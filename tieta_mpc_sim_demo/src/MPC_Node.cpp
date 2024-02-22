#include "tieta_mpc_sim_demo/MPC_Node.h"
using namespace std;

MPCNode::MPCNode()
{
    // Private parameters handler  私有的参数处理器，走的是mpc_node应该
    ros::NodeHandle pn("~");

    // Parameters for control loop
    pn.param("/dynamic_collision_mpc/thread_numbers", _thread_numbers, 2);   // 多线程的数量 number of threads for this ROS node
    pn.param("/dynamic_collision_mpc/debug_info", _debug_info, true);
    pn.param("/dynamic_collision_mpc/delay_mode", _delay_mode, true); //? 延迟模式？
    pn.param("/dynamic_collision_mpc/max_speed", _max_speed, 0.50);   // unit: m/s
    pn.param("/dynamic_collision_mpc/controller_freq", _controller_freq, 100); // 控制器的频率
    pn.param("/dynamic_collision_mpc/pid_freq", _pid_freq, 10);
    _dt = double(1.0 / _controller_freq); // time step duration dt in s 时间步长，0.1s
    pn.param<std::string>("/dynamic_collision_mpc/robot_description", _robot_description,
        "/home/diamondlee/VKConTieta_ws/src/urdf_description/robot_description/ridgeback_dual_arm_description/urdf/vkc_big_task.urdf.urdf");

    _collision_check = Collision_Check(_robot_description);
    _mpc = MPC(_collision_check);

    // Parameter for MPC solver 目标函数各项的惩罚系数
    pn.param("/dynamic_collision_mpc/mpc_steps", _mpc_steps, 21); // MPC的步长
    pn.param("/dynamic_collision_mpc/mpc_w_distx", _w_distx, 5000.0);
    pn.param("/dynamic_collision_mpc/mpc_w_disty", _w_disty, 5000.0);
    pn.param("/dynamic_collision_mpc/mpc_w_etheta", _w_etheta, 5000.0);
    pn.param("/dynamic_collision_mpc/mpc_w_vel", _w_vel, 1.0); //底盘的速度
    pn.param("/dynamic_collision_mpc/mpc_w_angvel", _w_angvel, 100.0); //底盘的角速度
    pn.param("/dynamic_collision_mpc/mpc_w_jnt", _w_jnt, 5000.0); //关节角的位置差，权重系数
    pn.param("/dynamic_collision_mpc/mpc_w_jntvel", _w_jntvel, 100.0); //关节角的速度，惩罚系数
    pn.param("/dynamic_collision_mpc/collision_base_weight", _w_base_collision, 1000.0);
    pn.param("/dynamic_collision_mpc/collision_shoulder_weight", _w_shoulder_collision, 1000.0);
    pn.param("/dynamic_collision_mpc/collision_elbow_weight", _w_elbow_collision, 1000.0);
    pn.param("/dynamic_collision_mpc/collision_wrist_weight", _w_wrist_collision, 1000.0);
    pn.param("/dynamic_collision_mpc/collision_gripper_weight", _w_gripper_collision, 1000.0);

    //目标函数的参数
    pn.param("/dynamic_collision_mpc/base_threshold", _base_threshold, 0.05);
    pn.param("/dynamic_collision_mpc/shoulder_threshold", _shoulder_threshold, 0.05);
    pn.param("/dynamic_collision_mpc/elbow_threshold", _elbow_threshold, 0.05);
    pn.param("/dynamic_collision_mpc/wrist_threshold", _wrist_threshold, 0.05);
    pn.param("/dynamic_collision_mpc/gripper_threshold", _gripper_threshold, 0.05);
    pn.param("/dynamic_collision_mpc/pedestrian_threshold", _pedestrian_threshold, 0.05);
    pn.param("/dynamic_collision_mpc/pedestrain_velocity", _pedestrian_vel, 0.05);

    pn.param("/dynamic_collision_mpc/mpc_max_vel", _max_speed, 1.0);
    pn.param("/dynamic_collision_mpc/mpc_max_angvel", _max_angvel, 3.0);
    pn.param("/dynamic_collision_mpc/mpc_max_jntvel", _max_jntvel, 3.0);
    pn.param("/dynamic_collision_mpc/mpc_bound_value", _bound_value, 1.0e3); // Bound value for other variables
    pn.param("/dynamic_collision_mpc/angel_upper_bound", _angel_upper, M_PI);
    pn.param("/dynamic_collision_mpc/angle_lower_bound", _angel_lower, -M_PI);//底盘的theta

    // 机械臂的角度限制
    pn.param("/dynamic_collision_mpc/joint1_upper_bound", _joint1_upper, M_PI);
    pn.param("/dynamic_collision_mpc/joint1_lower_bound", _joint1_lower, -M_PI);
    pn.param("/dynamic_collision_mpc/joint2_upper_bound", _joint2_upper, M_PI);
    pn.param("/dynamic_collision_mpc/joint2_lower_bound", _joint2_lower, -M_PI);
    pn.param("/dynamic_collision_mpc/joint3_upper_bound", _joint3_upper, M_PI);
    pn.param("/dynamic_collision_mpc/joint3_lower_bound", _joint3_lower, -M_PI);
    pn.param("/dynamic_collision_mpc/joint4_upper_bound", _joint4_upper, M_PI);
    pn.param("/dynamic_collision_mpc/joint4_lower_bound", _joint4_lower, -M_PI);
    pn.param("/dynamic_collision_mpc/joint5_upper_bound", _joint5_upper, M_PI);
    pn.param("/dynamic_collision_mpc/joint5_lower_bound", _joint5_lower, -M_PI);
    pn.param("/dynamic_collision_mpc/joint6_upper_bound", _joint6_upper, M_PI);
    pn.param("/dynamic_collision_mpc/joint6_lower_bound", _joint6_lower, -M_PI);

    //到达初始位置的tolerence
    pn.param("/dynamic_collision_mpc/tolerence_position", _tolerence_xy, 0.01);
    pn.param("/dynamic_collision_mpc/tolerence_theta", _tolerence_theta, 0.005);
    pn.param("/dynamic_collision_mpc/tolerence_joint", _tolerence_joint, 0.005);

    // PID参数 omega就是所有和角度相关的pid
    pn.param("/dynamic_collision_mpc/x_kp", _kp_vx, 0.2);
    pn.param("/dynamic_collision_mpc/x_ki", _ki_vx, 0.01);
    pn.param("/dynamic_collision_mpc/x_kd", _kd_vx, 0.005);

    pn.param("/dynamic_collision_mpc/y_kp", _kp_vy, 0.2);
    pn.param("/dynamic_collision_mpc/y_ki", _ki_vy, 0.01);
    pn.param("/dynamic_collision_mpc/y_kd", _kd_vy, 0.005);

    pn.param("/dynamic_collision_mpc/yaw_kp", _kp_omega, 0.3);
    pn.param("/dynamic_collision_mpc/yaw_ki", _ki_omega, 0.02);
    pn.param("/dynamic_collision_mpc/yaw_kd", _kd_omega, 0.01);

    //#####用于获取数据,存放的文件夹的path
    pn.param<std::string>("/dynamic_collision_mpc/savefolder_path", save_folder_path, " ");
    pn.param<std::string>("/dynamic_collision_mpc/save_Debugfolder_path", save_Debugfolder_path, " ");

    //*****************************************************************************
    // Display the parameters
    cout << "\n===== Parameters =====" << endl;
    cout << "/dynamic_collision_mpc/debug_info: " << _debug_info << endl;
    cout << "/dynamic_collision_mpc/delay_mode: " << _delay_mode << endl;
    cout << "/dynamic_collision_mpc/frequency_dT: " << _dt << endl;
    cout << "/dynamic_collision_mpc/mpc_steps: " << _mpc_steps << endl;
    cout << "/dynamic_collision_mpc/mpc_w_distx: " << _w_distx << endl;
    cout << "/dynamic_collision_mpc/mpc_w_disty: " << _w_disty << endl;
    cout << "/dynamic_collision_mpc/mpc_w_etheta: " << _w_etheta << endl;
    cout << "/dynamic_collision_mpc/mpc_max_angvel: " << _max_angvel << endl;
    cout << "/dynamic_collision_mpc/mpc_max_vel: " << _max_speed << endl;

    // Publishers and Subscribers
    _sub_timed_traj = _nh.subscribe("/traj_topic", 1, &MPCNode::rcvJointTrajCB, this);
    _sub_robot_state = _nh.subscribe("/joint_states", 1, &MPCNode::getRobotStateCB, this);
    _pub_robot_velocity = _nh.advertise<std_msgs::Float64MultiArray>("/joint_velocity", 1);

    //*******************************************************************************
    // pub mec reach init flag
    _pub_initPose_reach = _nh.advertise<std_msgs::Float32>("tieta_init_flag", 10);

    //*******************************************************************************
    // Init variables
    _traj_received = false;
    _track_finished = false;
    _trackTraj_computed = false;

    _loop_count = 0;
    _realTraj_length = 0;
    _subTraj_length = 0;

    _jntvel_msg = std_msgs::Float64MultiArray();
    _mpc_traj = JointTrajPub::AnglesList();

    // Init parameters for MPC object
    _mpc_params["DT"] = _dt;
    _mpc_params["STEPS"] = _mpc_steps;
    //目标函数惩罚系数
    _mpc_params["W_DISTX"] = _w_distx;
    _mpc_params["W_DISTY"] = _w_disty;
    _mpc_params["W_EPSI"] = _w_etheta;
    _mpc_params["W_VEL"] = _w_vel;
    _mpc_params["W_ANGVEL"] = _w_angvel;
    _mpc_params["W_JOINT"] = _w_jnt;
    _mpc_params["W_JNTVEL"] = _w_jntvel;
    _mpc_params["COLLISION_BASE_WEIGHT"] = _w_base_collision;
    _mpc_params["COLLISION_SHOULDER_WEIGHT"] = _w_shoulder_collision;
    _mpc_params["COLLISION_ELBOW_WEIGHT"] = _w_elbow_collision;
    _mpc_params["COLLISION_WRIST_WEIGHT"] = _w_wrist_collision;
    _mpc_params["COLLISION_GRIPPER_WEIGHT"] = _w_gripper_collision;

    _mpc_params["BASE_THRESHOLD"] = _base_threshold;
    _mpc_params["SHOULDER_THRESHOLD"] = _shoulder_threshold;
    _mpc_params["ELBOW_THRESHOLD"] = _elbow_threshold;
    _mpc_params["WRIST_THRESHOLD"] = _wrist_threshold;
    _mpc_params["GRIPPER_THRESHOLD"] = _gripper_threshold;
    _mpc_params["PEDESTRIAN_THRESHOLD"] = _pedestrian_threshold;
    _mpc_params["PEDESTRIAN_VELOCITY"] = _pedestrian_vel;

    _mpc_params["MAXVEL"] = _max_speed;
    _mpc_params["MAX_ANGVEL"] = _max_angvel;
    _mpc_params["MAX_JNTVEL"] = _max_jntvel;
    _mpc_params["BOUND"] = _bound_value;
    //底盘的theta上下界
    _mpc_params["ANGEL_UPPER"] = _angel_upper;
    _mpc_params["ANGEL_LOWER"] = _angel_lower;
    //机械臂UR的上下限
    _mpc_params["JOINT1_UPPER"] = _joint1_upper;
    _mpc_params["JOINT1_LOWER"] = _joint1_lower;
    _mpc_params["JOINT2_UPPER"] = _joint2_upper;
    _mpc_params["JOINT2_LOWER"] = _joint2_lower;
    _mpc_params["JOINT3_UPPER"] = _joint3_upper;
    _mpc_params["JOINT3_LOWER"] = _joint3_lower;
    _mpc_params["JOINT4_UPPER"] = _joint4_upper;
    _mpc_params["JOINT4_LOWER"] = _joint4_lower;
    _mpc_params["JOINT5_UPPER"] = _joint5_upper;
    _mpc_params["JOINT5_LOWER"] = _joint5_lower;
    _mpc_params["JOINT6_UPPER"] = _joint6_upper;
    _mpc_params["JOINT6_LOWER"] = _joint6_lower;

    //*************************************************************************
    //这里来创建文件夹 获取当前的日期和时间
    auto _time_now = std::chrono::system_clock::now();
    auto _in_time_t = std::chrono::system_clock::to_time_t(_time_now);
    //使用put_time 格式化日期和身体的
    std::stringstream _folder_ss;
    _folder_ss << std::put_time(std::localtime(&_in_time_t), "%Y%m%d_%H%M%S");

    //构建文件夹的名称
    const std::string foldername = save_folder_path + "/sim_plot_" + _folder_ss.str();
    //创建文件夹,并检验文件夹是否创建
    if(!boost::filesystem::create_directory(foldername))
        ROS_ERROR("can`t creat the record folder: %s", foldername.c_str());
    else
        ROS_INFO("Create the record folder: %s", foldername.c_str());
    //创建CSV文件名
    const std::string record_filename = foldername + "/exe_traj.csv";
    //打开文件
    file = std::ofstream(record_filename,ios::app);
    //file.open(record_filename,ios::app);
    if(file.is_open())
        ROS_INFO("record_csv/csv file has been open !");
    else
        ROS_ERROR("Cannot open record file: %s", record_filename.c_str());
    //-----------------record的文件夹和路径创建完毕---------------------------


    //-----------------开始debug的文件夹和路径---------------------------------
    const std::string debugfoldername = save_Debugfolder_path + "/sim_debug" + _folder_ss.str();
    //创建文件夹,并检验文件夹是否创建
    if(!boost::filesystem::create_directory(debugfoldername))
        ROS_ERROR("can`t creat the record folder: %s", debugfoldername.c_str());
    else
        ROS_INFO("Create the record folder: %s", debugfoldername.c_str());
    //创建CSV文件名
    const std::string debug_filename = debugfoldername + "/debug_exe_traj.csv";
    //打开文件
    file_debug = std::ofstream(debug_filename,ios::app);
    //file_debug.open(debug_filename);
    if(file_debug.is_open())
        ROS_INFO("debug_csv/csv file has been open !");
    else
        ROS_ERROR("Cannot open debug file: %s", debug_filename.c_str());
    //---------------debug的文件夹和路径创建完毕-----------------------------

    //--------------- 给MPC_Controller的结果(预测结果)指定存放的文件夹路径----------
    _mpc._file_path_class_MPC = foldername;
    // debug里是包含文字说明的
    _mpc._file_debug_path_class_MPC = debugfoldername;
    //--------------- 给FG_eval的结果(预测时域内的轨迹值)指定存放的文件夹路径--------
    _mpc._file_debug_path_class_FG_eval = debugfoldername;

    _mpc.LoadParams(_mpc_params);
    //todo需要新的变量来存储每一个的cost，和当前的distance。
    _mpc_etheta = 0;
    _mpc_distx = 0;
    _mpc_disty = 0;

    _rcv_traj = JointTrajPub::AnglesList();

    //*******************************************************************************
    //首先，机器人和行人的速度都为0
    for (unsigned int i = 0; i < 10; i++)
    {
        _jntvel_msg.data[i] = 0.0; //机器人9自由度加上行人1个自由度
    }
    //由于test_sim.cpp中，发布了速度信息，所以不需要last_position
    //获取行人和各个sphere的初始位姿 /tf
    try
    {
        //debug 需要有个wait,不然一开始也会加载
        _tfListener.waitForTransform("world", "dynamic_pedestrian", ros::Time(0), ros::Duration(1.0));
        _tfListener.lookupTransform("world", "dynamic_pedestrian", ros::Time(0), _tfTransform);
        // 如果拿到了tf，将位姿赋给行人状态
        dynamic_pedestrian_pos << _tfTransform.getOrigin().getX(), _tfTransform.getOrigin().getY(), _tfTransform.getOrigin().getZ();

        _tfListener.waitForTransform("world", "front_left_sphere_collision", ros::Time(0), ros::Duration(1.0));
        _tfListener.lookupTransform("world", "front_left_sphere_collision", ros::Time(0), _tfTransform);
        base_lf_sphere_pos << _tfTransform.getOrigin().getX(), _tfTransform.getOrigin().getY(), _tfTransform.getOrigin().getZ();

        _tfListener.waitForTransform("world", "front_right_sphere_collision", ros::Time(0), ros::Duration(1.0));
        _tfListener.lookupTransform("world", "front_right_sphere_collision", ros::Time(0), _tfTransform);
        base_rf_sphere_pos << _tfTransform.getOrigin().getX(), _tfTransform.getOrigin().getY(), _tfTransform.getOrigin().getZ();

        _tfListener.waitForTransform("world", "rear_left_sphere_collision", ros::Time(0), ros::Duration(1.0));
        _tfListener.lookupTransform("world", "rear_left_sphere_collision", ros::Time(0), _tfTransform);
        base_lr_sphere_pos << _tfTransform.getOrigin().getX(), _tfTransform.getOrigin().getY(), _tfTransform.getOrigin().getZ();

        _tfListener.waitForTransform("world", "rear_right_sphere_collision", ros::Time(0), ros::Duration(1.0));
        _tfListener.lookupTransform("world", "rear_right_sphere_collision", ros::Time(0), _tfTransform);
        base_rr_sphere_pos << _tfTransform.getOrigin().getX(), _tfTransform.getOrigin().getY(), _tfTransform.getOrigin().getZ();

        _tfListener.waitForTransform("world", "right_arm_shoulder_sphere_link", ros::Time(0), ros::Duration(1.0));
        _tfListener.lookupTransform("world", "right_arm_shoulder_sphere_link", ros::Time(0), _tfTransform);
        shoulder_sphere_pos << _tfTransform.getOrigin().getX(), _tfTransform.getOrigin().getY(), _tfTransform.getOrigin().getZ();

        _tfListener.waitForTransform("world", "right_arm_forearm_sphere_link", ros::Time(0), ros::Duration(1.0));
        _tfListener.lookupTransform("world", "right_arm_forearm_sphere_link", ros::Time(0), _tfTransform);
        elbow_sphere_pos << _tfTransform.getOrigin().getX(), _tfTransform.getOrigin().getY(), _tfTransform.getOrigin().getZ();

        _tfListener.waitForTransform("world", "right_arm_wrist_sphere_link", ros::Time(0), ros::Duration(1.0));
        _tfListener.lookupTransform("world", "right_arm_wrist_sphere_link", ros::Time(0), _tfTransform);
        wrist_sphere_pos << _tfTransform.getOrigin().getX(), _tfTransform.getOrigin().getY(), _tfTransform.getOrigin().getZ();

        _tfListener.waitForTransform("world", "right_arm_flange_gripper_sphere", ros::Time(0), ros::Duration(1.0));
        _tfListener.lookupTransform("world", "right_arm_flange_gripper_sphere", ros::Time(0), _tfTransform);
        gripper_sphere_pos << _tfTransform.getOrigin().getX(), _tfTransform.getOrigin().getY(), _tfTransform.getOrigin().getZ();

        ROS_WARN("The Obstacle-Class got the inital pose!");
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
    }

    file << "loop_cont" << " , "
             << "time_coordinate" << " , "
             << "robot_x_" << " , "
             << "robot_y_" << " , "
             << "robot_yaw" << " , "
             << "pre_bot speed_x" << " , "
             << "pre_bot speed_y" << " , "
             << "Pre_bot angvel"  << " , "
             << "real_bot speed_x" << " , "
             << "real_bot speed_y" << " , "
             << "real_bot angvel" << " , "
             << "world speed_x" << " , "
             << "world speed_y" << " , "
             << "world angvel" << " , "
             << "loop_time_cost" << " , "
             << "loop_x_Tcost" << " , "
             << "loop_y_Tcost" << " , "
             << "loop_theta_Tcost" << " , "
             << "loop_acc_x_Tcost" << " , "
             << "loop_acc_y_Tcost" << " , "
             << "loop_acc_theta_Tcost" << endl;
    _time_start = ros::Time::now().toSec();
}

MPCNode::~MPCNode()
{
    file.close();
};
void MPCNode::getRobotStateCB(const sensor_msgs::JointStateConstPtr &robotstateMsg)
{
    std::lock_guard<std::mutex> lock(mlock);
    _joint_pos = robotstateMsg->position;
    _joint_vel = robotstateMsg->velocity;
}

void MPCNode::setJointVelocity(const std::vector<double> &jointVelocity)
{
    for (unsigned int i = 0; i < jointVelocity.size(); i++)
    {
        _jntvel_msg.data[i] = jointVelocity[i];
    }
}
// Public: return _thread_numbers
int MPCNode::get_thread_numbers()
{
    return _thread_numbers;
}

// Public: return controll_freq
int MPCNode::get_controll_freq()
{
    return _controller_freq;
}

// 将角度规定在[-pi,pi]
double MPCNode::range_angle_PI(double angle)
{
    if (angle > M_PI)
        angle -= 2 * M_PI;
    else if (angle <= -M_PI)
        angle += 2 * M_PI;
    return angle;
}

// 将速度规定在[-max_speed , max_speed]
double MPCNode::range_velocity_MAX(double _input_v)
{
    if (_input_v >= _max_speed)
        _input_v = _max_speed;

    if (_input_v <= -_max_speed)
        _input_v = -_max_speed;

    return _input_v;
}

// 将角速度规定在[-max_angvel , max_angvel]
double MPCNode::range_angvel_MAX(double _input_w)
{
    if (_input_w >= _max_angvel)
        _input_w = _max_angvel;

    if (_input_w <= -_max_angvel)
        _input_w = -_max_angvel;

    return _input_w;
}

//todo 修改使得可以完成9自由度 先让机器人到达第一个轨迹点附近
bool MPCNode::gotoInitState()
{
    // 延时以等待traj to pub,并且订阅
    ros::Duration(0.5).sleep();

    vector<double> first_pose;
    // 再次判断
    while (!_traj_received)
        ROS_INFO("Waiting for the track Traj ...");
    ROS_INFO("rcvTrajCB has filled MPCNode._rcv_traj");

    // 得到第一个trajPoint
    JointTrajPub::AnglesList rcvTraj_msg = _rcv_traj;
    first_pose.push_back(rcvTraj_msg.AnglesList[0].base_x);
    first_pose.push_back(rcvTraj_msg.AnglesList[0].base_y);
    first_pose.push_back(rcvTraj_msg.AnglesList[0].base_theta);
    first_pose.push_back(rcvTraj_msg.AnglesList[0].joint1);
    first_pose.push_back(rcvTraj_msg.AnglesList[0].joint2);
    first_pose.push_back(rcvTraj_msg.AnglesList[0].joint3);
    first_pose.push_back(rcvTraj_msg.AnglesList[0].joint4);
    first_pose.push_back(rcvTraj_msg.AnglesList[0].joint5);
    first_pose.push_back(rcvTraj_msg.AnglesList[0].joint6);

    std::vector<double> current_pos;
    std::vector<double> current_vel;

    ros::Rate gotoRate(_pid_freq);
    double pid_dt = 1 / _pid_freq;
    bool pid_loop = true;
    while (pid_loop)
    {
        try
        {
            {
                std::lock_guard<std::mutex> lock(mlock);
                current_pos = _joint_pos;
            }
            _sensor_get = true;
            // 如果拿到了tf，将位姿赋给机器人状态
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            _sensor_get = false;
            continue;
        }
        // ref - current
        double error_x = first_pose[0] - _joint_pos[0];
        double error_y = first_pose[1] - _joint_pos[1];
        double error_theta = first_pose[2] - _joint_pos[2];
        double error_joint1 = first_pose[3] - _joint_pos[3];
        double error_joint2 = first_pose[4] - _joint_pos[4];
        double error_joint3 = first_pose[5] - _joint_pos[5];
        double error_joint4 = first_pose[6] - _joint_pos[6];
        double error_joint5 = first_pose[7] - _joint_pos[7];
        double error_joint6 = first_pose[8] - _joint_pos[8];

        double error_distance = sqrt(pow(error_x, 2) + pow(error_y, 2));
        bool _flag = error_distance <= _tolerence_xy && error_theta <= _tolerence_theta && error_joint1 <= _tolerence_joint
                        && error_joint2 <= _tolerence_joint && error_joint3 <= _tolerence_joint && error_joint4 <= _tolerence_joint
                            && error_joint5 <= _tolerence_joint && error_joint6 <= _tolerence_joint;

        if (_flag)
        {
            // 如果到达期望的轨迹的初始位置，速度置为0
            setJointVelocity({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
            _pub_robot_velocity.publish(_jntvel_msg);
            // 到达期望位置后，跳出循环，结束gotoInitState
            pid_loop = false;
            _sensor_get = false;
            ros::Duration(2.0).sleep(); // 3s缓一会儿
        }
        else
        {
            std::vector<double> temp_vel;
            // pi control 比例积分控制 && normalize
            temp_vel.push_back(range_velocity_MAX(_kp_vx * error_x + _ki_vx * error_x * pid_dt));
            temp_vel.push_back(range_velocity_MAX(_kp_vy * error_y + _ki_vy * error_y * pid_dt));
            temp_vel.push_back(range_angvel_MAX(_kp_omega * error_theta + _ki_omega * error_theta * pid_dt));

            temp_vel.push_back(range_angvel_MAX(_kp_omega * error_joint1 + _ki_omega * error_joint1 * pid_dt));
            temp_vel.push_back(range_angvel_MAX(_kp_omega * error_joint2 + _ki_omega * error_joint2 * pid_dt));
            temp_vel.push_back(range_angvel_MAX(_kp_omega * error_joint3 + _ki_omega * error_joint3 * pid_dt));
            temp_vel.push_back(range_angvel_MAX(_kp_omega * error_joint4 + _ki_omega * error_joint4 * pid_dt));
            temp_vel.push_back(range_angvel_MAX(_kp_omega * error_joint5 + _ki_omega * error_joint5 * pid_dt));
            temp_vel.push_back(range_angvel_MAX(_kp_omega * error_joint6 + _ki_omega * error_joint6 * pid_dt));

            setJointVelocity(temp_vel);
            _pub_robot_velocity.publish(_jntvel_msg);
        }
        // rate sleep 进入定时循环
        gotoRate.sleep();
    }
    return true;
}


//todo 消息类型修改为sensor_msgs::JointState
void MPCNode::rcvJointTrajCB(const JointTrajPub::AnglesListConstPtr &totalTrajMsg)
{
    if (_rcv_traj.AnglesList.size() == 0)
    {
        _rcv_traj = *totalTrajMsg; // pose的消息类型的vector，有时间戳和坐标

        _subTraj_length = _rcv_traj.AnglesList.size(); //订阅到的加长轨迹，末尾有重复的
        if (_subTraj_length != 0)
        {
            if (_delay_mode)
            {
                ROS_WARN("Delay mode is on");
                _realTraj_length = _subTraj_length - (_mpc_steps - 2) - 1;
            }
            else
            {
                ROS_WARN("Delay mode is off");
                _realTraj_length = _subTraj_length - (_mpc_steps - 2); //订阅到，本身traj的长度
            }
            cout << "rcvTrajCB中mpcsteps为 " << _mpc_steps << endl;
            ROS_INFO("Track Traj has received !!! Track Traj real length: %d", _realTraj_length);
            ROS_INFO("Track Traj has received !!! Track Traj subscribed length: %d", _subTraj_length);
            _traj_received = true;
        }
        else
        {
            ROS_ERROR("No trajectory form parseCSV_node is received");
            _traj_received = false;
        }
        // 已收到目标点，但是还没到
    }
    return;
}

JointTrajPub::AnglesList MPCNode::getTrackTraj(const JointTrajPub::AnglesList &rcvJointTrajMsg)
{
    _track_finished = false;
    // 声明一个空的path，留给MPC，
    JointTrajPub::AnglesList get_trackTraj = JointTrajPub::AnglesList(); // For generating mpc reference path
    // 给一个param，在traj_pub中重复给N次最后一个的traj点，保证能以速度为0到达终点
    // 有一个判断，判断loop_count是否和_rcv_traj.size()-_mpc_steps相同

    // new 与之前的跟踪路线不同，预测区间段是_mpc_steps - 1，但是算误差的轨迹点的数量还是_mpc_steps
    // 要是有_delay_mode的话，取的是_loop_count+1的trajPoint作为第一个
    int _fir_track_point;
    if (_delay_mode)
        _fir_track_point = _loop_count + 1;
    else
        _fir_track_point = _loop_count;

    for (int i = 0; i < _mpc_steps; i++)
    {
        JointTrajPub::Angles temp_jntpos;
        temp_jntpos.base_x = rcvJointTrajMsg.AnglesList[i + _fir_track_point].base_x;
        temp_jntpos.base_y = rcvJointTrajMsg.AnglesList[i + _fir_track_point].base_y;
        temp_jntpos.base_theta = rcvJointTrajMsg.AnglesList[i + _fir_track_point].base_theta;

        temp_jntpos.joint1 = rcvJointTrajMsg.AnglesList[i + _fir_track_point].joint1;
        temp_jntpos.joint2 = rcvJointTrajMsg.AnglesList[i + _fir_track_point].joint2;
        temp_jntpos.joint3 = rcvJointTrajMsg.AnglesList[i + _fir_track_point].joint3;
        temp_jntpos.joint4 = rcvJointTrajMsg.AnglesList[i + _fir_track_point].joint4;
        temp_jntpos.joint5 = rcvJointTrajMsg.AnglesList[i + _fir_track_point].joint5;
        temp_jntpos.joint6 = rcvJointTrajMsg.AnglesList[i + _fir_track_point].joint6;

        get_trackTraj.AnglesList.push_back(temp_jntpos);
        // 放入第loop_count次预测过程，需要跟踪的参考轨迹
    }
    _trackTraj_computed = true;

    return get_trackTraj;
}


// Rater: Control Loop (closed loop nonlinear MPC)RATE保证周期性：控制环（闭环非线性 MPC）

bool MPCNode::controlLoop()
{

    double time_get_tf_start;
    double time_get_tf_end;
    time_get_tf_start = ros::Time::now().toSec();
    cout << "***********************  开始Control Loop  ************************" << endl;
    cout << "_traj_received为" << _traj_received << endl;
    cout << "_track_finished为" << _track_finished << endl;
    cout << "_loop_count为" << _loop_count << endl;
    cout << "_realTraj_length为" << _realTraj_length << endl;

    //std::vector<Eigen::Vector3d> _tf_state;

    try
    {
        //debug 需要有个wait,不然一开始也会加载
        _tfListener.lookupTransform("world", "dynamic_pedestrian", ros::Time(0), _tfTransform);
        // 如果拿到了tf，将位姿赋给行人状态
        dynamic_pedestrian_pos << _tfTransform.getOrigin().getX(), _tfTransform.getOrigin().getY(), _tfTransform.getOrigin().getZ();

        _tfListener.lookupTransform("world", "front_left_sphere_collision", ros::Time(0), _tfTransform);
        base_lf_sphere_pos << _tfTransform.getOrigin().getX(), _tfTransform.getOrigin().getY(), _tfTransform.getOrigin().getZ();

        _tfListener.lookupTransform("world", "front_right_sphere_collision", ros::Time(0), _tfTransform);
        base_rf_sphere_pos << _tfTransform.getOrigin().getX(), _tfTransform.getOrigin().getY(), _tfTransform.getOrigin().getZ();

        _tfListener.lookupTransform("world", "rear_left_sphere_collision", ros::Time(0), _tfTransform);
        base_lr_sphere_pos << _tfTransform.getOrigin().getX(), _tfTransform.getOrigin().getY(), _tfTransform.getOrigin().getZ();

        _tfListener.lookupTransform("world", "rear_right_sphere_collision", ros::Time(0), _tfTransform);
        base_rr_sphere_pos << _tfTransform.getOrigin().getX(), _tfTransform.getOrigin().getY(), _tfTransform.getOrigin().getZ();

        _tfListener.lookupTransform("world", "right_arm_shoulder_sphere_link", ros::Time(0), _tfTransform);
        shoulder_sphere_pos << _tfTransform.getOrigin().getX(), _tfTransform.getOrigin().getY(), _tfTransform.getOrigin().getZ();

        _tfListener.lookupTransform("world", "right_arm_forearm_sphere_link", ros::Time(0), _tfTransform);
        elbow_sphere_pos << _tfTransform.getOrigin().getX(), _tfTransform.getOrigin().getY(), _tfTransform.getOrigin().getZ();

        _tfListener.lookupTransform("world", "right_arm_wrist_sphere_link", ros::Time(0), _tfTransform);
        wrist_sphere_pos << _tfTransform.getOrigin().getX(), _tfTransform.getOrigin().getY(), _tfTransform.getOrigin().getZ();

        _tfListener.lookupTransform("world", "right_arm_flange_gripper_sphere", ros::Time(0), _tfTransform);
        gripper_sphere_pos << _tfTransform.getOrigin().getX(), _tfTransform.getOrigin().getY(), _tfTransform.getOrigin().getZ();

        _tf_state.push_back(dynamic_pedestrian_pos);
        _tf_state.push_back(base_lf_sphere_pos);
        _tf_state.push_back(base_rf_sphere_pos);
        _tf_state.push_back(base_lr_sphere_pos);
        _tf_state.push_back(base_rr_sphere_pos);
        _tf_state.push_back(shoulder_sphere_pos);
        _tf_state.push_back(elbow_sphere_pos);
        _tf_state.push_back(wrist_sphere_pos);
        _tf_state.push_back(gripper_sphere_pos);

        ROS_WARN("Update the Obstacle-Class pose!");
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
    }

    // 收到要跟踪的轨迹，收到小车的tf位姿，轨迹还没跟踪完，control loop没有到最后一个点
    bool _run_loop_traj = _traj_received && !_track_finished;
    bool _run_loop_count = _loop_count < _realTraj_length - 1;
    bool _run_loop = _run_loop_traj && _run_loop_count;

    cout << "_run_loop_traj为 " << _run_loop_traj << endl;
    cout << "_run_loop_count为 " << _run_loop_count << endl;
    cout << "_run_loop为 " << _run_loop << endl;

    double time_get_TrackTraj_start;
    double time_get_TrackTraj_end;

    if (_run_loop)
    {
        // 里程计信息 在vicon_bridge中是tf
        JointTrajPub::AnglesList rcvTrajmsg = _rcv_traj; // mpc_path，通过desire_path处理过的

        time_get_TrackTraj_start = ros::Time::now().toSec();

        // 现在对_rcv_traj进行处理，得到用于本次loop要track的轨迹
        JointTrajPub::AnglesList mpc_trackTraj = getTrackTraj(rcvTrajmsg);

        time_get_TrackTraj_end = ros::Time::now().toSec();
        cout << "getTrackTraj用时: " << time_get_TrackTraj_end - time_get_TrackTraj_start << endl;
        cout << "getTrackTraj stamp: " << time_get_TrackTraj_end - time_get_tf_start << endl;

        // read 已经获得了机器人的位姿，以及要跟踪的轨迹信息。
        // 加上个判断_trackTraj_computed
        bool _mpc_trackTraj_sizeFlag = (mpc_trackTraj.AnglesList.size() == _mpc_steps);
        cout << "_mpc_trackTraj_size是否正确: " << _mpc_trackTraj_sizeFlag << endl;
        cout << "_trackTraj_computed为 " << _trackTraj_computed << endl;
        bool _start_mpc_trackTraj = _mpc_trackTraj_sizeFlag && _trackTraj_computed;
        cout << "_start_mpc_trackTraj为 " << _start_mpc_trackTraj << endl;

        if (_start_mpc_trackTraj)
        {
            cout << "===================  进入前期计算  =============================" << endl;
            double time_loop_start = ros::Time::now().toSec();
            _loop_count++;
            //! Update system states: X=[x, y, yaw, theta1-7]
            //! Update system inputs: U=[vx, vy, w, w1-7]
            // 一开始这些都是0.0
            const double dt = _dt; // 0.01s
            //! 这里就不用这么麻烦的计算了，就用回调函数得到的，把位置和速度都给放进去。
            std::vector<double> current_position;
            std::vector<double> current_velocity;
            try{
                std::lock_guard<std::mutex> lock(mlock);
                current_position = _joint_pos;
                current_velocity = _joint_vel;
            }
            catch (tf::TransformException &ex)
            {
                ROS_ERROR("%s", ex.what());
            }

            Eigen::VectorXd robot_state(18); //当前位置9和当前速度9，其实直接控制这9个Joint，都是线性方程
            if (!_delay_mode)
            {
                // Kinematic model is used to predict vehicle state at the actual moment of control (current time + delay dt)
                // 利用运动学模型预测实际控制时刻的车辆状态（当前时间+延迟dt）
                robot_state << current_position[0], current_position[1], current_position[2], current_position[3], current_position[4],
                                current_position[5], current_position[6], current_position[7], current_position[8],
                                current_velocity[0], current_velocity[1], current_velocity[2], current_velocity[3], current_velocity[4],
                                current_velocity[5], current_velocity[6], current_velocity[7], current_velocity[8];
            }
            else
            {
                ROS_ERROR("Donot Open the Delay Mode ! ! !");
            }

            double time_solve_start = ros::Time::now().toSec();
            vector<double> mpc_results = _mpc.Solve(robot_state, mpc_trackTraj, _tf_state);
            double time_solve_end = ros::Time::now().toSec();
            cout << "MPC solve time: " << time_solve_end - time_solve_start << " s" << endl;
            cout << "MPC solve Stamp: " << time_solve_end - time_get_tf_start << endl;

            // normalize speed & angvel
            _speed_x = range_velocity_MAX(mpc_results[0]);
            _speed_y = range_velocity_MAX(mpc_results[1]);
            _angvel = range_angvel_MAX(mpc_results[2]);
            _jntvel1 = range_angvel_MAX(mpc_results[3]);
            _jntvel2 = range_angvel_MAX(mpc_results[4]);
            _jntvel3 = range_angvel_MAX(mpc_results[5]);
            _jntvel4 = range_angvel_MAX(mpc_results[6]);
            _jntvel5 = range_angvel_MAX(mpc_results[7]);
            _jntvel6 = range_angvel_MAX(mpc_results[8]);
            //如果MPC求解失败，mpc_result[9] = 0
            if(mpc_results[9] < 0.5)
                _loop_count = _loop_count - 1;

            // print INFO
            if (_debug_info)
            {

            }

                // 用于计算耗时
            double time_loop_end = ros::Time::now().toSec();
            loop_duration = time_loop_end - time_loop_start;
            _time_coordinate = time_loop_end - _time_start;
            cout << "loop_时间戳: " << time_loop_end - time_get_tf_start << endl;
            cout << "***********************  结束Control Loop  ************************" << endl;
            // cout << "\n" << endl;
            cout << "\n" << endl;
        }
        else
            ROS_WARN("MPC trajectory for trackTraj is not got");
    }
    else
    {
        //_acc = 0.0;
        _speed_x = 0.0;
        _speed_y = 0.0;
        _angvel = 0.0;
        _jntvel1 = 0.0;
        _jntvel2 = 0.0;
        _jntvel3 = 0.0;
        _jntvel4 = 0.0;
        _jntvel5 = 0.0;
        _jntvel6 = 0.0;
        // 其实等于就够了
        if (_traj_received)
        {
            if (_loop_count > _realTraj_length - 2 && _loop_count > 0 && _rcv_traj.AnglesList.size() > 0)
                _track_finished = true;
            if (_track_finished && _traj_received)
                ROS_ERROR("Traj has Tracked: control loop OOOVER !");
        }
    }

    // todo 在这里debug，csv文件的输出
    {
        // cout << "loop_cont time is " << _loop_count << " , "
        //      << "time coordinate is " << _time_coordinate << " , "
        //      << "robot_x position is " << _rb_x << " , "
        //      << "robot_y position is " << _rb_y << " , "
        //      << "robot_yaw is " << _rb_theta << " , "
        //      << "pre_bot speed_x is " << _speed_x << " , "
        //      << "pre_bot speed_y is " << _speed_y << " , "
        //      << "Pre_bot angvel is " << _angvel << " , "
        //      << "real_bot speed_x is " << _vel_bot[0] << " , "
        //      << "real_bot speed_y is " << _vel_bot[1] << " , "
        //      << "real_bot angvel is " << _vel_bot[2] << " , "
        //      << "world speed_x is " << _vel_map[0] << " , "
        //      << "world speed_y is " << _vel_map[1] << " , "
        //      << "world angvel is " << _vel_map[2] << " , "
        //      << "this loop x_direciton total cost: " << _mpc._mpc_distx_Tcost << " , "
        //      << "this loop y_direciton total cost: " << _mpc._mpc_disty_Tcost << " , "
        //      << "this loop theta total cost: " << _mpc._mpc_etheta_Tcost << " , "
        //      << "this loop acc_x total cost: " << _mpc._mpc_acc_x_Tcost << " , "
        //      << "this loop acc_y total cost: " << _mpc._mpc_acc_y_Tcost << " , "
        //      << "this loop acc_theta total cost: " << _mpc._mpc_angacc_Tcost << " , "
        //      << "this loop time_cost is " << loop_duration << endl;
        // cout << "\n" << endl;
        // cout << "\n" << endl;

        // file << std::fixed
        //      << std::setprecision(std::numeric_limits<double>::max_digits10)
        //      << _loop_count << ","
        //      << _time_coordinate << ","
        //      << _rb_x << ","
        //      << _rb_y << ","
        //      << _rb_theta << ","
        //      << _speed_x << ","
        //      << _speed_y << ","
        //      << _angvel << ","
        //      << _vel_bot[0] << ","
        //      << _vel_bot[1] << ","
        //      << _vel_bot[2] << ","
        //      << _vel_map[0] << ","
        //      << _vel_map[1] << ","
        //      << _vel_map[2] << ","
        //      << loop_duration << ","
        //      << _mpc._mpc_distx_Tcost << ","
        //      << _mpc._mpc_disty_Tcost << ","
        //      << _mpc._mpc_etheta_Tcost << ","
        //      << _mpc._mpc_acc_x_Tcost << ","
        //      << _mpc._mpc_acc_y_Tcost << ","
        //      << _mpc._mpc_angacc_Tcost << endl;
    }
    if(_debug_info)
    {
        // file_debug << std::fixed
        //         << std::setprecision(std::numeric_limits<double>::max_digits10)
        //         << "loop_cont time is " << _loop_count << " , "
        //         << "time coordinate is " << _time_coordinate << " , "
        //         << "robot_x position is " << _rb_x << " , "
        //         << "robot_y position is " << _rb_y << " , "
        //         << "robot_yaw is " << _rb_theta << " , "
        //         << "robot speed_x is " << _speed_x << " , "
        //         << "robot speed_y is " << _speed_y << " , "
        //         << "robot angvel is " << _angvel << " , "
        //         << "this loop x_direciton total cost: " << _mpc._mpc_distx_Tcost << " , "
        //         << "this loop y_direciton total cost: " << _mpc._mpc_disty_Tcost << " , "
        //         << "this loop theta total cost: " << _mpc._mpc_etheta_Tcost << " , "
        //         << "this loop acc_x total cost: " << _mpc._mpc_acc_x_Tcost << " , "
        //         << "this loop acc_y total cost: " << _mpc._mpc_acc_y_Tcost << " , "
        //         << "this loop acc_theta total cost: " << _mpc._mpc_angacc_Tcost << " , "
        //         << "this loop time_cost is " << loop_duration << endl;
    }

    // publish general cmd_vel
    if (!_track_finished)
    {
        _jntvel_msg.data.push_back(_speed_x);
        _jntvel_msg.data.push_back(_speed_y);
        _jntvel_msg.data.push_back(_angvel);
        _jntvel_msg.data.push_back(_jntvel1);
        _jntvel_msg.data.push_back(_jntvel2);
        _jntvel_msg.data.push_back(_jntvel3);
        _jntvel_msg.data.push_back(_jntvel4);
        _jntvel_msg.data.push_back(_jntvel5);
        _jntvel_msg.data.push_back(_jntvel6);
        _jntvel_msg.data.push_back(_pedestrian_vel);

        _pub_robot_velocity.publish(_jntvel_msg);
        double time_pub_cmd = ros::Time::now().toSec();
        cout << "pub的时间戳" << time_pub_cmd - time_get_tf_start << endl;
    }
    else
    {
        // 这个也没有必要
        _jntvel_msg.data = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, _pedestrian_vel};
        _pub_robot_velocity.publish(_jntvel_msg);
        if(true)
            ROS_WARN("track is finished");
        else
            ROS_ERROR("pub twist flag is false");
        return false;
    }
    return true;
}
