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
    pnh.param("/trajPub/delay_mode", _delay_mode, true);
    pnh.param("/trajPub/mpc_steps", _expand_num, 10);
    pnh.param("/trajPub/controller_freq", _loop_Hz, 100);
    pnh.param<std::string>("/trajPub/map_frame", _map_frame, "world");
    pnh.param<std::string>("/trajPub/csvfile_path", _csv_file, "");

    cout<< _map_frame <<endl;

    vector<map<double, vector<double>>> traj2pub =
        parseCSV2Map(_csv_file, _expand_num, _delay_mode, _loop_Hz);
    ROS_INFO("STARTING parseCSV !!!   traj2pub size: %d", traj2pub[0].size());

    ros::Publisher traj_pub = pnh.advertise<nav_msgs::Path>("traj_topic", 10);
    ros::Publisher anglesList_pub = pnh.advertise<anglesPub::AnglesList>("anglesList_topic", 10);

    cout << "here1" << endl;
    ros::Rate loop_rate(10); // 发布频率 1 Hz

    while (ros::ok())
    {
        cout << "here2" << endl;
        nav_msgs::Path traj_msg;

        traj_msg.header.frame_id = _map_frame; // 设置坐标系
        traj_msg.header.stamp = ros::Time::now(); // 设置时间戳

        geometry_msgs::PoseStamped pose;

        // for base
        for (map<double, vector<double>>::iterator iter = traj2pub[0].begin(); iter!=traj2pub[0].end();iter++)
        {
            cout << " x is "<< iter->second[0] << " y is " << iter->second[1] << " theta is " << iter->second[2] << endl;
            pose.header.stamp = ros::Time::now();
            pose.header.frame_id = _map_frame;
            pose.header.seq = static_cast<std::uint32_t>(iter->first*100);

            pose.pose.position.x = iter->second[0];
            pose.pose.position.y = iter->second[1];
            //todo 用z来存储csv中的期望theta
            pose.pose.position.z = iter->second[2];

            traj_msg.poses.push_back(pose);
            //break;
        }
        cout << "here3" << endl;
        anglesPub::AnglesList anglesList_msg;
        anglesPub::Angles angles;
        // for 7-dof arm
        for(map<double, vector<double>>::iterator iter = traj2pub[1].begin(); iter!=traj2pub[1].end();iter++)
        {
            //一个时刻的角度值vector
            angles.joint1=iter->second[0];
            angles.joint2=iter->second[1];
            angles.joint3=iter->second[2];
            angles.joint4=iter->second[3];
            angles.joint5=iter->second[4];
            angles.joint6=iter->second[5];
            angles.joint7=iter->second[6];

            anglesList_msg.AnglesList.push_back(angles);
        }
        //angles_pub.publish(anglesList_msg);

        cout << "traj_msg.poses.size() is " << traj_msg.poses.size() << endl;
        cout << "anglesList_msg.AnglesList.size()" << anglesList_msg.AnglesList.size() << endl;

        if(traj_msg.poses.size() != 0)
            traj_pub.publish(traj_msg);
        if(anglesList_msg.AnglesList.size() != 0)
            anglesList_pub.publish(anglesList_msg);

        ros::spinOnce();

        loop_rate.sleep();
        //cout << "执行了一次" << endl;
    }

    return 0;
}