#include "tieta_mpc_sim_demo/FG_evalue.h"
#include "tieta_mpc_sim_demo/MPC_Controller.h"
#include "tieta_mpc_sim_demo/MPC_Node.h"
#include "tieta_mpc_sim_demo/Collision_Check.h"

/*****************/
/* MAIN FUNCTION */
/*****************/
int main(int argc, char **argv)
{
    //Initiate ROS
    //命名空间，最后还是以launch文件为主，MPC_tracking
    ros::init(argc, argv, "MPC_dynamic_collision_node");
    //常用写法，进入类中，利用构造函数
    ros::NodeHandle main_nh;
    // 等待初始化
    ros::Duration(2.0).sleep();
    //放在了MPCNode内，ros::Duration(1.0).sleep(); // 延时1.0s，等待gazebo和rviz的初始化

    MPCNode mpc_node;
    //在这启动mpc_node的controll_loop函数

    //设置循环的频率
    ros::Rate controlloop_Rate(mpc_node.get_controll_freq());

    ROS_INFO("Waiting for reference traj msgs ~");
    ros::AsyncSpinner spinner(mpc_node.get_thread_numbers()); // Use multi threads
    spinner.start();

    ros::Duration(1.0).sleep();

    ROS_WARN("Start Spinning ! & Controll to Init Traj Point !!!!!!");

    if(mpc_node.gotoInitState())
        ROS_INFO("if code ,Arrived Init State !!!!!!");

    ros::Duration(1.0).sleep();

    ROS_WARN("Arrived Init State , Press 'Enter' to Start Controll Loop !!!!!!");

    std::cin.get();
    std::cout << "debug_start" << std::endl;

    mpc_node._time_start = ros::Time::now().toSec();

    while (ros::ok())
    {
        //here to pub
        //t0计算出的速度结果，作用在t1上（指令有延迟的话，就会延迟执行）
        //控制器，里面计算cmd_vel并发布，需要考虑的是在什么位置发布速度指令
        if(!mpc_node.controlLoop())
            ros::shutdown();
        // ros::spinOnce();  已经有spinner.start()
        //  here to pub
        //  放在这里，t0计算的结果，延迟执行到t1上
        controlloop_Rate.sleep();
        //here to pub
        //和放在开头执行没有区别
    }

    ros::waitForShutdown();
    return 0;
}
