#include "open_door_tieta_mpc/FG_evalue.h"
#include "open_door_tieta_mpc/MPC_Controller.h"
#include "open_door_tieta_mpc/MPC_Node.h"
//#include "open_door_tieta_mpc/Collision_Check.h"

//需要构建一个全局的Veloity-list
template<typename T>
using joints_velocity = std::vector<T>;
//TODO 这里就不需要这么多了，只需要给base和行人发送数据，行人的话可能需要两个方向的速度，这就需要给urdf了，先假设只有一个方向的速度
std::vector<joints_velocity<double>> Velocity_Solution(50, joints_velocity<double>(4, 0.0)); //3 joints + 行人的速度
bool update_velocity_solution = false;
std::mutex update_mutex;

//publish thread
void publish_thread_jointVelocitys(ros::Publisher &pub, ros::NodeHandle &nh){
    ros::Rate rate(10);
    int i = 0;
    while(ros::ok()){
        std_msgs::Float64MultiArray single_velocity_msg;
        {
            std::lock_guard<std::mutex> lock(update_mutex);
            if(update_velocity_solution)
            {
                update_velocity_solution = false;
                i = 0;
            }
        }
        {
            single_velocity_msg.data = Velocity_Solution[i];
            pub.publish(single_velocity_msg);
            std::cout << "publish joint velocitys: " << Velocity_Solution[i][1] << "," <<
            Velocity_Solution[i][2] << "," << Velocity_Solution[i][3] << "," <<
            Velocity_Solution[i][4] << "," << Velocity_Solution[i][5] << "," <<
            Velocity_Solution[i][6] << "," << Velocity_Solution[i][7] << "," <<
            Velocity_Solution[i][8] << "," << Velocity_Solution[i][9] << "," <<
            std::endl;
            i++;

            rate.sleep();
            ros::spinOnce();
        }
    }
}
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
    ros::Duration(1.0).sleep();
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

    ros::Publisher pub = main_nh.advertise<std_msgs::Float64MultiArray>("/joint_velocity", 1);
    std::thread publish_thread(publish_thread_jointVelocitys, std::ref(pub), std::ref(main_nh));

    ROS_WARN("Arrived Init State , Press 'Enter' to Start Controll Loop !!!!!!");

    std::cin.get();
    std::cout << "debug_start" << std::endl;



    mpc_node._time_start = ros::Time::now().toSec();
    bool track_continue_flag = true;

    while (ros::ok())
    {
        // //here to pub
        // //t0计算出的速度结果，作用在t1上（指令有延迟的话，就会延迟执行）
        // //控制器，里面计算cmd_vel并发布，需要考虑的是在什么位置发布速度指令
        // if(!mpc_node.controlLoop())
        //     ros::shutdown();
        // // ros::spinOnce();  已经有spinner.start()
        // //  here to pub
        // //  放在这里，t0计算的结果，延迟执行到t1上
        // controlloop_Rate.sleep();
        // //here to pub
        // //和放在开头执行没有区别
        //debug
        {
            std::vector<joints_velocity<double>> temp_solutions(50, std::vector<double>(4, 0.0));
            std::vector<joints_velocity<double>> temp_;
            temp_ = mpc_node.controlLoop(track_continue_flag);

        // std::cout <<
        //     "debug_end" << std::endl;

            std::transform(temp_.begin(), temp_.end(), temp_solutions.begin(), temp_solutions.begin(),
                            [](const std::vector<double> &source, std::vector<double> &target) {
                                std::copy(source.begin(), source.end(), target.begin());
                                return target;});

            Velocity_Solution = temp_solutions;

            // std::cout <<
            //     "debug_end#############################" << std::endl;
        }
        {
            std::lock_guard<std::mutex> lock(update_mutex);
            update_velocity_solution = true;
        }

        controlloop_Rate.sleep();
    }

    publish_thread.join();

    ros::waitForShutdown();
    return 0;
}
