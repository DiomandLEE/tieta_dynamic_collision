#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <parseCSV/parseCSV.h>

using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "traj_publisher_node");
    ros::NodeHandle pnh;
    int _expand_num;
    bool _delay_mode;
    int _loop_Hz;
    string _map_frame;
    string _csv_file;
    pnh.param("/DoorTrajPub/dynamic_collision_mpc/delay_mode", _delay_mode, false);
    pnh.param("/DoorTrajPub/dynamic_collision_mpc/increase_length", _expand_num, 10);
    pnh.param("controller_freq", _loop_Hz, 100);

    pnh.param<std::string>("/DoorTrajPub/dynamic_collision_mpc/csvfile_path", _csv_file, "/home/diamondlee/VKConTieta_ws/src/tieta_mpc_sim_demo/tieta_track_traj/place/retimed_placeTraj_15_36_51.csv");
    std::cout << "csvfile_path: " << std::endl;
    vector<map<double, vector<double>>> traj2pub =
        parseCSV2Map(_csv_file, _expand_num, _delay_mode, _loop_Hz);
    std::cout << "traj2pub size: " << traj2pub.size() << std::endl;
    ROS_INFO("\n STARTING parseCSV !!!   traj2pub size: %d", traj2pub[0].size());

    ros::Publisher anglesList_pub = pnh.advertise<DoorTrajPub::AnglesList>("door_anglesList_topic", 1);

    ros::Rate loop_rate(10); // 发布频率 1 Hz

    while (ros::ok())
    {
        DoorTrajPub::AnglesList anglesList_msg;
        DoorTrajPub::Angles angles;
        // for 7-dof arm
        for(map<double, vector<double>>::iterator iter = traj2pub[0].begin(); iter!=traj2pub[0].end();iter++)
        {
            angles.base_x = iter->second[0];
            angles.base_y = iter->second[1];
            angles.base_theta = iter->second[2];
            // 一个时刻的角度值vector
            angles.joint1 = iter->second[3];
            angles.joint2 = iter->second[4];
            angles.joint3 = iter->second[5];
            angles.joint4 = iter->second[6];
            angles.joint5 = iter->second[7];
            angles.joint6 = iter->second[8];
            angles.joint_door = iter->second[9];

            anglesList_msg.AnglesList.push_back(angles);
        }
        //angles_pub.publish(anglesList_msg);

        if(traj2pub[0].size() != 0)
            anglesList_pub.publish(anglesList_msg);

        ros::spinOnce();

        loop_rate.sleep();
        //cout << "执行了一次" << endl;
    }

    return 0;
}