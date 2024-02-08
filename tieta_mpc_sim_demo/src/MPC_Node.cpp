#include "tieta_mpc_sim_demo/MPC_Node.h"

MPCNode::MPCNode()
{
    // Private parameters handler  私有的参数处理器，走的是mpc_node应该
    ros::NodeHandle pn("~");

    // ros::Duration(2.0).sleep(); // 延时1.0s，等待gazebo和rviz的初始化

    // Parameters for control loop
    pn.param("/mec_mpc/thread_numbers", _thread_numbers, 2);   // 多线程的数量 number of threads for this ROS node
    pn.param("/mec_mpc/pub_twist_cmd", _pub_twist_flag, true); // 发布转向的cmd
    pn.param("/mec_mpc/debug_info", _debug_info, true);
    pn.param("/mec_mpc/delay_mode", _delay_mode, true); //? 延迟模式？
    pn.param("/mec_mpc/max_speed", _max_speed, 0.50);   // unit: m/s
    // pn.param("/mec_mpc/waypoints_dist", _waypointsDist, -1.0); // unit: m //-1的意思是由节点计算
    // pn.param("/mec_mpc/path_length", _pathLength, 2.0); // unit: m
    // pn.param("/mec_mpc/goal_radius", _goalRadius, 0.5); // unit: m
    pn.param("/mec_mpc/controller_freq", _controller_freq, 100); // 控制器的频率
    pn.param("/mec_mpc/pid_freq", _pid_freq, 10);
    // pn.param("/mec_mpc/vehicle_Lf", _Lf, 0.290); // distance between the front of the vehicle and its center of gravity
    _dt = double(1.0 / _controller_freq); // time step duration dt in s 时间步长，0.1s

    // Parameter for MPC solver
    pn.param("/mec_mpc/mpc_steps", _mpc_steps, 21); // MPC的步长
    // pn.param("/mec_mpc/mpc_ref_distx", _ref_distx, 0.0);  //cte 与道路中心的偏差，横向误差
    // pn.param("/mec_mpc/mpc_ref_vel", _ref_vel, 1.0);  //速度
    // pn.param("/mec_mpc/mpc_ref_etheta", _ref_etheta, 0.0); //与道路的theta偏差，角度误差
    pn.param("/mec_mpc/mpc_w_distx", _w_distx, 5000.0);
    pn.param("/mec_mpc/mpc_w_disty", _w_disty, 5000.0);
    pn.param("/mec_mpc/mpc_w_etheta", _w_etheta, 5000.0);
    pn.param("/mec_mpc/mpc_w_vel", _w_vel, 1.0);
    pn.param("/mec_mpc/mpc_w_angvel", _w_angvel, 100.0);
    pn.param("/mec_mpc/mpc_w_angacc", _w_angacc, 10.0);
    pn.param("/mec_mpc/mpc_w_acc", _w_acc, 50.0);
    // pn.param("/mec_mpc/mpc_w_jerk", _w_jerk, 10.0);
    pn.param("/mec_mpc/mpc_max_angvel", _max_angvel, 3.0);     // Maximal angvel radian (~30 deg)
    pn.param("/mec_mpc/mpc_max_vel", _max_speed, 1.0);         // Maximal accel
    pn.param("/mec_mpc/mpc_bound_value", _bound_value, 1.0e3); // Bound value for other variables
    pn.param("/mec_mpc/angel_upper_bound", _angel_upper, M_PI);
    pn.param("/mec_mpc/angle_lower_bound", _angel_lower, -M_PI);
    pn.param("/mec_mpc/tolerence_position", _tolerence_xy, 0.01);
    pn.param("/mec_mpc/tolerence_angle", _tolerence_theta, 0.005);

    pn.param("/mec_mpc/x_kp", _kp_vx, 0.2);
    pn.param("/mec_mpc/x_ki", _ki_vx, 0.01);
    pn.param("/mec_mpc/x_kd", _kd_vx, 0.005);

    pn.param("/mec_mpc/y_kp", _kp_vy, 0.2);
    pn.param("/mec_mpc/y_ki", _ki_vy, 0.01);
    pn.param("/mec_mpc/y_kd", _kd_vy, 0.005);

    pn.param("/mec_mpc/yaw_kp", _kp_omega, 0.3);
    pn.param("/mec_mpc/yaw_ki", _ki_omega, 0.02);
    pn.param("/mec_mpc/yaw_kd", _kd_omega, 0.01);

    //#####用于获取数据,存放的文件夹的path
    pn.param<std::string>("/mec_mpc/savefolder_path", save_folder_path, " ");
    pn.param<std::string>("/mec_mpc/save_Debugfolder_path", save_Debugfolder_path, " ");

    // pn.param<std::string>("/mec_mpc/save_Debugfile_path_Class_MPC", save_Debugfile_path_MPC, " ");
    // pn.param<std::string>("/mec_mpc/save_Debugfile_path_Class_FG_eval", save_Debugfile_path_FG_eval, " ");

    // Parameter for topics & Frame name
    // pn.param<std::string>("/mec_mpc/global_path_topic", _globalPath_topic, "/move_base/TrajectoryPlannerROS/global_plan" );
    // pn.param<std::string>("/mec_mpc/goal_topic", _goal_topic, "/move_base_simple/goal" );
    pn.param<std::string>("/mec_mpc/map_frame", _map_frame, "world"); //*****for mpc, "odom"
    // pn.param<std::string>("/mec_mpc/odom_frame", _odom_frame, "odom");
    pn.param<std::string>("/mec_mpc/car_frame", _car_frame, "base_footprint");

    //*******************************************************************************
    // init flag mec_reach
    pn.param("/mec_mpc/kinova_reach_init_flag",_kinova_reach_init,false);

    // Display the parameters
    cout << "\n===== Parameters =====" << endl;
    cout << "/mec_mpc/pub_twist_cmd: " << _pub_twist_flag << endl;
    cout << "/mec_mpc/debug_info: " << _debug_info << endl;
    cout << "/mec_mpc/delay_mode: " << _delay_mode << endl;
    // cout << "vehicle_Lf: "  << _Lf << endl;
    cout << "/mec_mpc/frequency_dT: " << _dt << endl;
    cout << "/mec_mpc/mpc_steps: " << _mpc_steps << endl;
    // cout << "mpc_ref_vel: " << _ref_vel << endl;
    cout << "/mec_mpc/mpc_w_distx: " << _w_distx << endl;
    cout << "/mec_mpc/mpc_w_disty: " << _w_disty << endl;
    cout << "/mec_mpc/mpc_w_etheta: " << _w_etheta << endl;
    cout << "/mec_mpc/mpc_max_angvel: " << _max_angvel << endl;
    cout << "/mec_mpc/mpc_max_vel: " << _max_speed << endl;

    // Publishers and Subscribers
    //_sub_odom   = _nh.subscribe("/odom", 1, &MPCNode::odomCB, this);
    //_sub_path   = _nh.subscribe( _globalPath_topic, 1, &MPCNode::pathCB, this);//空的
    _sub_timed_traj = _nh.subscribe("traj_topic", 1, &MPCNode::rcvtrajCB, this); // 收到这个话题之后，发布"/mpc_reference"
    //_sub_goal   = _nh.subscribe( _goal_topic, 1, &MPCNode::goalCB, this); //记录目标goal的位姿
    //_sub_amcl   = _nh.subscribe("/amcl_pose", 5, &MPCNode::amclCB, this);

    //_pub_odompath  = _nh.advertise<nav_msgs::Path>("/mpc_reference", 1); // reference path for MPC ///mpc_reference
    _pub_mpctraj = _nh.advertise<nav_msgs::Path>("/mpc_trajectory", 1); // MPC trajectory output
    //_pub_ackermann = _nh.advertise<ackermann_msgs::AckermannDriveStamped>("/ackermann_cmd", 1);
    if (_pub_twist_flag)
        _pub_twist = _nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1); // for stage (Ackermann msg non-supported)

    _pub_totalcost = _nh.advertise<std_msgs::Float32>("/total_cost", 1);         // Global path generated from another source
    _pub_distx_cost = _nh.advertise<std_msgs::Float32>("/distx_track_error", 1); // Global path generated from another source
    _pub_etheta_cost = _nh.advertise<std_msgs::Float32>("/theta_error", 1);      // Global path generated from another source

    // init pose
    //*******************************************************************************
    // pub mec reach init flag
    _pub_initPose_reach = _nh.advertise<std_msgs::Float32>("mec_init_flag", 10);
    // subscribe kinova reach init flag
    _sub_kinova_reach_init = _nh.subscribe("kinova_init_flag", 1, &MPCNode::kinova_reach_init_callback, this);

    //*******************************************************************************
    // init reach Pose flag
    pn.param("/mec_mpc/mec_reach_init_flag",_mec_reach_init,false);
    //_mec_reach_init = false;
    //_kinova_reach_init = false;
    // Timer
    // 定时器，定时执行
    //_timer_mpc = _nh.createTimer(ros::Duration((1.0)/_controller_freq), &MPCNode::controlLoopCB, this); // 10Hz //*****mpc

    // Init variables
    _traj_received = false;
    _track_finished = false;
    _trackTraj_computed = false;

    _speed_x = 0.0;
    _speed_y = 0.0;
    _angvel = 0.0;
    //
    _loop_count = 0;
    _realTraj_length = 0;
    _subTraj_length = 0;

    //_ackermann_msg = ackermann_msgs::AckermannDriveStamped();
    _twist_msg = geometry_msgs::Twist();
    _mpc_traj = nav_msgs::Path();

    // Init parameters for MPC object
    _mpc_params["DT"] = _dt;
    //_mpc_params["LF"] = _Lf;
    _mpc_params["STEPS"] = _mpc_steps;
    //_mpc_params["REF_DISTX"]  = _ref_distx;
    //_mpc_params["REF_ETHETA"] = _ref_etheta;
    //_mpc_params["REF_V"]    = _ref_vel;
    _mpc_params["W_DISTX"] = _w_distx;
    _mpc_params["W_DISTY"] = _w_disty;
    _mpc_params["W_EPSI"] = _w_etheta;
    _mpc_params["W_V"] = _w_vel;
    _mpc_params["W_ANGVEL"] = _w_angvel;
    _mpc_params["W_ACC"] = _w_acc;
    _mpc_params["W_DANGVEL"] = _w_angacc;
    //_mpc_params["W_JERK"]     = _w_jerk;
    _mpc_params["ANGVEL"] = _max_angvel;
    _mpc_params["MAXVEL"] = _max_speed;
    _mpc_params["BOUND"] = _bound_value;
    _mpc_params["ANGEL_UPPER"] = _angel_upper;
    _mpc_params["ANGEL_LOWER"] = _angel_lower;
    //_mpc_params["FILE_DEBUG_PATH"] = save_Debugfile_path;
    // 加载到刚才的系数到_mpc_params中

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
    // const std::string foldername_predict = foldername + "/predict_traj";
    // //创建文件夹,并检验文件夹是否创建
    // if(!boost::filesystem::create_directory(foldername_predict))
    //     ROS_ERROR("can`t creat the record folder: %s", foldername_predict.c_str());
    // else
    //     ROS_INFO("Create the record folder: %s", foldername_predict.c_str());
    //_mpc._file_path_class_MPC = foldername_predict;
    // debug里是包含文字说明的
    _mpc._file_debug_path_class_MPC = debugfoldername;
    //--------------- 给FG_eval的结果(预测时域内的轨迹值)指定存放的文件夹路径--------
    _mpc._file_debug_path_class_FG_eval = debugfoldername;

    _mpc.LoadParams(_mpc_params);

    // min_idx = 0;
    // idx = 0;
    _mpc_etheta = 0;
    _mpc_distx = 0;
    _mpc_disty = 0;

    _rcv_traj = {};

    //*******************************************************************************
    // init reach Pose flag
    //_mec_reach_init = false;
    //_kinova_reach_init = false;

    _last_rb_x = 0.0;
    _last_rb_y = 0.0;
    _last_rb_theta = 0.0;

    //! offset改为0.0
    _offset_matrix << cos(0.0), -sin(0.0), 0.0,
                        sin(0.0), cos(0.0), 0.0,
                        0.0 ,0.0, 1.0;

    //! 获取机器人的初始位姿 /tf
    try
    {
        //ros::Duration(0.1).sleep(); //debug 奇怪 加了这个就好了
        //debug 需要有个wait,不然一开始也会加载
        _tfListener.waitForTransform(_map_frame, _car_frame, ros::Time(0), ros::Duration(2.0));
        //_tfListener.lookupTransform(_car_frame, _map_frame, ros::Time(0), _tfTransform);
        _tfListener.lookupTransform(_map_frame, _car_frame, ros::Time(0), _tfTransform);
        // cout << "map frame " << _map_frame << endl;
        // cout << "car_frame " << _car_frame << endl;
        // 如果拿到了tf，将位姿赋给机器人状态
        _vicon_tf_x = _tfTransform.getOrigin().getX();
        _vicon_tf_y = _tfTransform.getOrigin().getY();
        _vicon_tf_theta = range_angle_PI(tf::getYaw(_tfTransform.getRotation()));
        ROS_INFO("VICON got the inital TF");

        // //开始坐标转换
        // _vicon_tf_matrix << cos(_vicon_tf_theta), -sin(_vicon_tf_theta), _vicon_tf_x,
        //                     sin(_vicon_tf_theta), cos(_vicon_tf_theta), _vicon_tf_y,
        //                     0.0, 0.0, 1.0;
        // _robot_tf_matrix = _vicon_tf_matrix * _offset_matrix;
        //_robot_tf_matrix = _offset_matrix * _vicon_tf_matrix;

        // _rb_x = _robot_tf_matrix(0,2);
        // _rb_y = _robot_tf_matrix(1,2);
        // _rb_theta = _vicon_tf_theta;
        _rb_x = _vicon_tf_x;
        _rb_y = _vicon_tf_y;
        _rb_theta = _vicon_tf_theta;

        _last_rb_x = _rb_x;
        _last_rb_y = _rb_y;
        _last_rb_theta = _rb_theta;

        ROS_INFO("The mec got the inital pose");
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

// 接收kinova reach init的flag
void MPCNode::kinova_reach_init_callback(const std_msgs::Float32::ConstPtr &msg)
{
    std::unique_lock<std::mutex> my_unique(mlock);
    _kinova_reach_init = true;
}

bool MPCNode::get_kinova_reach_flag()
{
    std::unique_lock<std::mutex> my_unique(mlock);
    bool rtn = _kinova_reach_init;
    //cout << "in mutex lock " << _kinova_reach_init << endl;
    return rtn;
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

// 先让机器人到达第一个轨迹点附近
bool MPCNode::gotoInitState()
{
    // 延时以等待traj to pub
    ros::Duration(0.5).sleep();

    vector<double> first_pose;
    // 再次判断
    while (!_traj_received)
        ROS_INFO("Waiting for the track Traj ...");
    ROS_INFO("rcvTrajCB has filled MPCNode._rcv_traj");

    // 得到第一个trajPoint
    nav_msgs::Path rcvTraj_msg = _rcv_traj;
    first_pose.push_back(rcvTraj_msg.poses[0].pose.position.x);
    first_pose.push_back(rcvTraj_msg.poses[0].pose.position.y);
    // z for theta
    first_pose.push_back(rcvTraj_msg.poses[0].pose.position.z);

    ros::Rate gotoRate(_pid_freq);
    double pid_dt = 1 / _pid_freq;
    bool pid_loop = true;
    while (pid_loop)
    {
        try
        {
            //_tfListener.lookupTransform(_car_frame, _map_frame, ros::Time(0), _tfTransform);
            _tfListener.lookupTransform(_map_frame, _car_frame, ros::Time(0), _tfTransform);
            // cout << "map frame " << _map_frame << endl;
            // cout << "car_frame " << _car_frame << endl;
            _tf_get = true;
            // 如果拿到了tf，将位姿赋给机器人状态
            _vicon_tf_x = _tfTransform.getOrigin().getX();
            _vicon_tf_y = _tfTransform.getOrigin().getY();
            _vicon_tf_theta = range_angle_PI(tf::getYaw(_tfTransform.getRotation()));
                            //开始坐标转换
            // _vicon_tf_matrix << cos(_vicon_tf_theta), -sin(_vicon_tf_theta), _vicon_tf_x,
            //                     sin(_vicon_tf_theta), cos(_vicon_tf_theta), _vicon_tf_y,
            //                     0.0, 0.0, 1.0;
            // _robot_tf_matrix = _vicon_tf_matrix * _offset_matrix;
            // //_robot_tf_matrix = _offset_matrix * _vicon_tf_matrix;

            // _rb_x = _robot_tf_matrix(0,2);
            // _rb_y = _robot_tf_matrix(1,2);
            // _rb_theta = _vicon_tf_theta;
            _rb_x = _vicon_tf_x;
            _rb_y = _vicon_tf_y;
            _rb_theta = _vicon_tf_theta;
            // cout << "vicon中的坐标" << _vicon_tf_x << "," << _vicon_tf_y << "," << _vicon_tf_theta << endl;
            // cout << "机器人的坐标" << _rb_x << "," << _rb_y << "," << _rb_theta  << endl;
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            _tf_get = false;
            continue;
        }
        // ref - current
        double error_x = first_pose[0] - _rb_x;
        double error_y = first_pose[1] - _rb_y;
        double error_theta = first_pose[2] - _rb_theta;

        double error_distance = sqrt(pow(error_x, 2) + pow(error_y, 2));
        //ROS_INFO("");

        if (error_distance <= _tolerence_xy && error_theta <= _tolerence_theta)
        {
            // 如果到达期望的轨迹的初始位置，速度置为0
            _twist_msg.linear.x = 0;
            _twist_msg.linear.y = 0;
            _twist_msg.angular.z = 0;
            _pub_twist.publish(_twist_msg);
            // 到达期望位置后，跳出循环，结束gotoInitState
            pid_loop = false;
            _tf_get = false;

            // todo 在这里加入flag
            ros::Duration(3.0).sleep(); // 3s缓一会儿
            _mec_reach_init = true;

            std_msgs::Float32 flag_msg;
            flag_msg.data = 1.0;
            for(int i = 0; i< 20; i++)
            {
                _pub_initPose_reach.publish(flag_msg);
                ros::Duration(0.05).sleep();
            }
        }
        else
        {
            // pi control 比例积分控制
            _speed_x = _kp_vx * error_x + _ki_vx * error_x * pid_dt;
            _speed_y = _kp_vy * error_y + _ki_vy * error_y * pid_dt;
            _angvel = _kp_omega * error_theta + _ki_omega * error_theta * pid_dt;
            // normalize
            _speed_x = range_velocity_MAX(_speed_x);
            _speed_y = range_velocity_MAX(_speed_y);
            _angvel = range_angvel_MAX(_angvel);

            _twist_msg.linear.x = _speed_x;
            _twist_msg.linear.y = _speed_y;
            _twist_msg.angular.z = _angvel;

            _pub_twist.publish(_twist_msg);
        }
        // rate sleep 进入定时循环
        gotoRate.sleep();
    }
    return true;
}
// CallBack: Update odometry
/*
void MPCNode::odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg)
{
    _odom = *odomMsg; //里程计的信息

}
*/
// CallBack: Update generated path (conversion to odom frame)

void MPCNode::rcvtrajCB(const nav_msgs::Path::ConstPtr &totalTrajMsg)
{
    if (_rcv_traj.poses.size() == 0)
    {
        _rcv_traj = *totalTrajMsg; // pose的消息类型的vector，有时间戳和坐标

        _subTraj_length = _rcv_traj.poses.size();
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
                _realTraj_length = _subTraj_length - (_mpc_steps - 2);
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

nav_msgs::Path MPCNode::getTrackTraj(const nav_msgs::Path &rcvTrajMsg)
{
    // todo 这个判断加到Controller——loop中，防止轨迹跟踪之后，控制器继续运行
    _track_finished = false;

    // 要有一个是变量告诉是在第几次的controllLoop中
    // todo 这个是放在类的privata中的 done
    // int loop_count = 0;

    // 声明一个空的path，留给MPC，
    nav_msgs::Path get_trackTraj = nav_msgs::Path(); // For generating mpc reference path
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
        geometry_msgs::PoseStamped tempPose;
        tempPose.pose.position.x = rcvTrajMsg.poses[i + _fir_track_point].pose.position.x;
        tempPose.pose.position.y = rcvTrajMsg.poses[i + _fir_track_point].pose.position.y;
        // z for theta
        tempPose.pose.position.z = rcvTrajMsg.poses[i + _fir_track_point].pose.position.z;
        tempPose.header.stamp = ros::Time::now();
        tempPose.header.frame_id = _map_frame;
        get_trackTraj.poses.push_back(tempPose);
        // 放入第loop_count次预测过程，需要跟踪的参考轨迹
    }
    _trackTraj_computed = true;

    return get_trackTraj;
}

// CallBack: Update path waypoints (conversion to odom frame)
/*
void MPCNode::pathCB(const nav_msgs::Path::ConstPtr& pathMsg)
{
}
*/

/*
// CallBack: Update goal status
void MPCNode::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg)
{
    _goal_pos = goalMsg->pose.position;
    _traj_received = true;
    _track_finished = false;
    ROS_INFO("Goal Received :goalCB!");
    //read 记录目标goal的位姿，将flag置位
}
*/

// Callback: Check if the car is inside the goal area or not检查汽车是否在目标区域内
/*
void MPCNode::amclCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amclMsg)
{

    if(_goal_received)
    {
        double car2goal_x = _goal_pos.x - amclMsg->pose.pose.position.x;
        double car2goal_y = _goal_pos.y - amclMsg->pose.pose.position.y;
        double dist2goal = sqrt(car2goal_x*car2goal_x + car2goal_y*car2goal_y);
        if(dist2goal < _goalRadius)
        {
            _goal_received = false;
            _goal_reached = true;
            _path_computed = false;
            ROS_INFO("Goal Reached !");
        }
        //检查到没到goal
    }

}
*/

// Rater: Control Loop (closed loop nonlinear MPC)RATE保证周期性：控制环（闭环非线性 MPC）

bool MPCNode::controlLoop()
{
    //_tfListener.lookupTransform(_car_frame, _map_frame, ros::Time(0), _tfTransform);
    //double time_loop_start = ros::Time::now().toSec();
    double time_get_tf_start;
    double time_get_tf_end;
    time_get_tf_start = ros::Time::now().toSec();
    cout << "***********************  开始Control Loop  ************************" << endl;
    try
    {
        // todo 在这里需要，利用差分运动学得到看，机器人的真实速度 20231030
        // 保存上一时刻的用于计算当前的速度
        _last_rb_x = _rb_x;
        _last_rb_y = _rb_y;
        _last_rb_theta = _rb_theta;

        //_tfListener.lookupTransform(_car_frame, _map_frame, ros::Time(0), _tfTransform);
        _tfListener.lookupTransform(_map_frame, _car_frame, ros::Time(0), _tfTransform);
        _tf_get = true;
        time_get_tf_end = ros::Time::now().toSec();
        cout << "tf_get_time_cost: " << time_get_tf_end - time_get_tf_start << endl;
        cout << "tf_get_time_stamp: " << time_get_tf_end - time_get_tf_start << endl;

        // 如果拿到了tf，将位姿赋给vicon_tf状态
        _vicon_tf_x = _tfTransform.getOrigin().getX();
        _vicon_tf_y = _tfTransform.getOrigin().getY();
        _vicon_tf_theta = range_angle_PI(tf::getYaw(_tfTransform.getRotation()));

        //todo 这里去掉坐标转换，直接拿/tf信息
                //开始坐标转换
        // _vicon_tf_matrix << cos(_vicon_tf_theta), -sin(_vicon_tf_theta), _vicon_tf_x,
        //                     sin(_vicon_tf_theta), cos(_vicon_tf_theta), _vicon_tf_y,
        //                     0.0, 0.0, 1.0;
        // _robot_tf_matrix = _vicon_tf_matrix * _offset_matrix;
        //_robot_tf_matrix = _offset_matrix * _vicon_tf_matrix;

        // _rb_x = _robot_tf_matrix(0,2);
        // _rb_y = _robot_tf_matrix(1,2);
        // _rb_theta = _vicon_tf_theta;
        _rb_x = _vicon_tf_x;
        _rb_y = _vicon_tf_y;
        _rb_theta = _vicon_tf_theta;
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
        _tf_get = false;
        //return false;  //这里不能return,否则直接推出main中的while循环了
    }
    cout << "tf_flag为" << _tf_get << endl;
    cout << "_mec_reach_init为" << _mec_reach_init << endl;
    cout << "_traj_received为" << _traj_received << endl;
    cout << "_track_finished为" << _track_finished << endl;
    cout << "_kinova_reach_flag为" << get_kinova_reach_flag() << endl;
    cout << "_loop_count为" << _loop_count << endl;
    cout << "_realTraj_length为" << _realTraj_length << endl;

    // 收到要跟踪的轨迹，收到小车的tf位姿，轨迹还没跟踪完，control loop没有到最后一个点
    bool _run_loop_reach = _mec_reach_init && get_kinova_reach_flag();
    bool _run_loop_traj = _traj_received && _tf_get && !_track_finished;
    bool _run_loop_count = _loop_count < _realTraj_length - 1;
    bool _run_loop = _run_loop_reach && _run_loop_traj && _run_loop_count;
    //< _realTraj_length - 1 received goal & goal not reached并且path已经算好了

    cout << "_run_loop_reach为 " << _run_loop_reach << endl;
    cout << "_run_loop_traj为 " << _run_loop_traj << endl;
    cout << "_run_loop_count为 " << _run_loop_count << endl;
    cout << "_run_loop为 " << _run_loop << endl;

    double time_get_TrackTraj_start;
    double time_get_TrackTraj_end;

    if (_run_loop)
    {
        // 里程计信息 在vicon_bridge中是tf
        nav_msgs::Path rcvTrajmsg = _rcv_traj; // mpc_path，通过desire_path处理过的

        time_get_TrackTraj_start = ros::Time::now().toSec();

        // 现在对_rcv_traj进行处理，得到用于本次loop要track的轨迹
        nav_msgs::Path mpc_trackTraj = getTrackTraj(rcvTrajmsg);

        time_get_TrackTraj_end = ros::Time::now().toSec();
        cout << "getTrackTraj用时: " << time_get_TrackTraj_end - time_get_TrackTraj_start << endl;
        cout << "getTrackTraj stamp: " << time_get_TrackTraj_end - time_get_tf_start << endl;

        // read 已经获得了机器人的位姿，以及要跟踪的轨迹信息。
        // 加上个判断_trackTraj_computed
        bool _mpc_trackTraj_sizeFlag = (mpc_trackTraj.poses.size() == _mpc_steps);
        cout << "_mpc_trackTraj_size是否正确: " << _mpc_trackTraj_sizeFlag << endl;
        cout << "_trackTraj_computed为 " << _trackTraj_computed << endl;
        bool _start_mpc_trackTraj = _mpc_trackTraj_sizeFlag && _trackTraj_computed;
        cout << "_start_mpc_trackTraj为 " << _start_mpc_trackTraj << endl;

        if (_start_mpc_trackTraj)
        {
            cout << "===================  进入前期计算  =============================" << endl;
            double time_loop_start = ros::Time::now().toSec();
            _loop_count++;

            //! Update system states: X=[x, y, theta] try&catch
            //! Update system inputs: U=[w, vx, vy]
            // 一开始这些都是0.0
            const double dt = _dt; // 0.01s
            // todo 就是在这里需修改速度 20231030 //done 之后也许会debug，涉及到-pi, pi的问题

            //debug 在这得到的就是robot在world系下的速度
            _vel_map[2] = (_rb_theta - _last_rb_theta) / dt; // steering -> w
            _vel_map[0] = (_rb_x - _last_rb_x) / dt;             // speed -> vx
            _vel_map[1] = (_rb_y - _last_rb_y) / dt;             // speed -> vy
            cout << "vx_map: " << _vel_map[0] << "vy_map" << _vel_map[1] << "angvel_map: " << _vel_map[2] << endl;

            //debug  利用旋转矩阵得到机器人的速度
            Eigen::Matrix3d Trans_map2bot;
            Trans_map2bot << cos(_rb_theta), -sin(_rb_theta), 0,
                                sin(_rb_theta), cos(_rb_theta), 0,
                                0, 0, 1;
            _vel_bot = Trans_map2bot.inverse() * _vel_map;

            cout << "vx_bot: " << _vel_bot[0] << "vy_bot" << _vel_bot[1] << "angvel_bot: " << _vel_bot[2] << endl;

            //!  state=[x, y, theta]
            // todo 之后设计一个，更改状态数量的parm，搭配launch
            Eigen::VectorXd state(6); // 6维的状态量
            if (_delay_mode)
            {
                // Kinematic model is used to predict vehicle state at the actual moment of control (current time + delay dt)
                // 利用运动学模型预测实际控制时刻的车辆状态（当前时间+延迟dt）
                const double px_act = _rb_x + _vel_map[0] * dt;
                const double py_act = _rb_y + _vel_map[1] * dt;
                const double theta_act = _rb_theta + _vel_map[2] * dt; // theta = theta + w * dt

                state << px_act, py_act, theta_act, _vel_bot[0], _vel_bot[1], _vel_bot[2];

                cout << "开启delay_mode! , 机器人在世界系中的状态为: " <<
                    "x = " << state[0] << " , y = " << state[1] << " , theta = " << state[2] << endl;

                file_debug << "开启delay_mode! , 机器人在世界系中的状态为: " <<
                    "x = " << state[0] << " , y = " << state[1] << " , theta = " << state[2] <<
                    " , vx_bot: " << _vel_bot[0] << " , vy_bot: " << _vel_bot[1] << " , angvel_bot: " << _vel_bot[2] << endl;
            }
            else
            {
                state << _rb_x, _rb_y, _rb_theta, _vel_bot[0], _vel_bot[1], _vel_bot[2];

                cout << "关闭delay_mode... , 机器人在世界系中的状态为: " <<
                    "x = " << state[0] << " , y = " << state[1] << " , theta = " << state[2] << endl;

                file_debug << "关闭delay_mode... , 机器人在世界系中的状态为: " <<
                    "x = " << state[0] << " , y = " << state[1] << " , theta = " << state[2] <<
                    " , vx_bot: " << _vel_bot[0] << " , vy_bot" << _vel_bot[1] << " , angvel_bot: " << _vel_bot[2] << endl;
            }
            // cout << "进入mpc.solve" << endl;

            double time_solve_start = ros::Time::now().toSec();
            vector<double> mpc_results = _mpc.Solve(state, mpc_trackTraj);
            //vector<double> mpc_results = _mpc.Solve(state_delay, mpc_trackTraj, _loop_count);
            //  cout << "出来mpc.solve" << endl;
            double time_solve_end = ros::Time::now().toSec();
            cout << "MPC solve time: " << time_solve_end - time_solve_start << " s" << endl;
            cout << "MPC solve Stamp: " << time_solve_end - time_get_tf_start << endl;
            // MPC result (all described in car frame), output = (vx, vy, w)
            _speed_x = mpc_results[0]; // m/s, longitudinal speed
            _speed_y = mpc_results[1]; // m/s, lateral speed
            _angvel = mpc_results[2];  // radian/sec, angular velocity

            // normalize speed & angvel
            _speed_x = range_velocity_MAX(_speed_x);
            _speed_y = range_velocity_MAX(_speed_y);
            _angvel = range_angvel_MAX(_angvel);

            // print INFO
            if (_debug_info)
            {
                cout << "\n\nDEBUG" << endl;
                cout << "robot_x: " << _rb_x << endl;
                cout << "robot_y: " << _rb_y << endl;
                cout << "robot_theta: " << _rb_theta << endl;
                    // cout << "V: " << v << endl;
                    // cout << "odom_path: \n" << odom_path << endl;
                    // cout << "x_points: \n" << x_veh << endl;
                    // cout << "y_points: \n" << y_veh << endl;
                    ////cout << "coeffs: \n" << coeffs << endl;
                cout << "_speed_x: \n"
                         << _speed_x << endl;
                cout << "_speed_y: \n"
                         << _speed_y << endl;
                cout << "_angvel: \n"
                         << _angvel << endl;
                // cout << "_accel: \n" << _acc << endl;

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
        _angvel = 0;
        // 其实等于就够了
        if (_traj_received)
        {
            if (_loop_count > _realTraj_length - 2 && _loop_count > 0 && _rcv_traj.poses.size() > 0)
                _track_finished = true;
            if (_track_finished && _traj_received)
                ROS_WARN("Traj has Tracked: control loop OOOVER !");
        }
    }

    // todo 在这里debug，csv文件的输出
    {
        cout << "loop_cont time is " << _loop_count << " , "
             << "time coordinate is " << _time_coordinate << " , "
             << "robot_x position is " << _rb_x << " , "
             << "robot_y position is " << _rb_y << " , "
             << "robot_yaw is " << _rb_theta << " , "
             << "pre_bot speed_x is " << _speed_x << " , "
             << "pre_bot speed_y is " << _speed_y << " , "
             << "Pre_bot angvel is " << _angvel << " , "
             << "real_bot speed_x is " << _vel_bot[0] << " , "
             << "real_bot speed_y is " << _vel_bot[1] << " , "
             << "real_bot angvel is " << _vel_bot[2] << " , "
             << "world speed_x is " << _vel_map[0] << " , "
             << "world speed_y is " << _vel_map[1] << " , "
             << "world angvel is " << _vel_map[2] << " , "
             << "this loop x_direciton total cost: " << _mpc._mpc_distx_Tcost << " , "
             << "this loop y_direciton total cost: " << _mpc._mpc_disty_Tcost << " , "
             << "this loop theta total cost: " << _mpc._mpc_etheta_Tcost << " , "
             << "this loop acc_x total cost: " << _mpc._mpc_acc_x_Tcost << " , "
             << "this loop acc_y total cost: " << _mpc._mpc_acc_y_Tcost << " , "
             << "this loop acc_theta total cost: " << _mpc._mpc_angacc_Tcost << " , "
             << "this loop time_cost is " << loop_duration << endl;
        cout << "\n" << endl;
        cout << "\n" << endl;

        file << std::fixed
             << std::setprecision(std::numeric_limits<double>::max_digits10)
             << _loop_count << ","
             << _time_coordinate << ","
             << _rb_x << ","
             << _rb_y << ","
             << _rb_theta << ","
             << _speed_x << ","
             << _speed_y << ","
             << _angvel << ","
             << _vel_bot[0] << ","
             << _vel_bot[1] << ","
             << _vel_bot[2] << ","
             << _vel_map[0] << ","
             << _vel_map[1] << ","
             << _vel_map[2] << ","
             << loop_duration << ","
             << _mpc._mpc_distx_Tcost << ","
             << _mpc._mpc_disty_Tcost << ","
             << _mpc._mpc_etheta_Tcost << ","
             << _mpc._mpc_acc_x_Tcost << ","
             << _mpc._mpc_acc_y_Tcost << ","
             << _mpc._mpc_angacc_Tcost << endl;
    }
    if(_debug_info)
    {
        file_debug << std::fixed
                << std::setprecision(std::numeric_limits<double>::max_digits10)
                << "loop_cont time is " << _loop_count << " , "
                << "time coordinate is " << _time_coordinate << " , "
                << "robot_x position is " << _rb_x << " , "
                << "robot_y position is " << _rb_y << " , "
                << "robot_yaw is " << _rb_theta << " , "
                << "robot speed_x is " << _speed_x << " , "
                << "robot speed_y is " << _speed_y << " , "
                << "robot angvel is " << _angvel << " , "
                << "this loop x_direciton total cost: " << _mpc._mpc_distx_Tcost << " , "
                << "this loop y_direciton total cost: " << _mpc._mpc_disty_Tcost << " , "
                << "this loop theta total cost: " << _mpc._mpc_etheta_Tcost << " , "
                << "this loop acc_x total cost: " << _mpc._mpc_acc_x_Tcost << " , "
                << "this loop acc_y total cost: " << _mpc._mpc_acc_y_Tcost << " , "
                << "this loop acc_theta total cost: " << _mpc._mpc_angacc_Tcost << " , "
                << "this loop time_cost is " << loop_duration << endl;
    }

    // publish general cmd_vel
    if (_pub_twist_flag && !_track_finished)
    {
        _twist_msg.linear.x = _speed_x;
        _twist_msg.linear.y = _speed_y;
        _twist_msg.angular.z = _angvel;

        _pub_twist.publish(_twist_msg);
        double time_pub_cmd = ros::Time::now().toSec();
        cout << "pub的时间戳" << time_pub_cmd - time_get_tf_start << endl;
    }
    else
    {
        // 这个也没有必要
        _twist_msg.linear.x = 0;
        _twist_msg.linear.y = 0;
        _twist_msg.angular.z = 0;
        _pub_twist.publish(_twist_msg);
        if(_pub_twist_flag)
            ROS_WARN("track is finished");
        else
            ROS_ERROR("pub twist flag is false");
        return false;
    }
    return true;
}
