#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <math.h>
#include <map>
#include <sstream>
#include <unistd.h>
#include <boost/multiprecision/cpp_dec_float.hpp>
#include <iomanip>
#include <limits>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

using namespace boost::multiprecision;

int main(int argc, char **argv){
    ros::init(argc, argv, "test_csv_pub_node");
    ros::NodeHandle nh;

    std::string file_in = "/home/diamondlee/VKConTieta_ws/src/tieta_mpc_sim_demo/tieta_track_traj/pick/retimed_pickTraj_15_36_51.csv";
    std::ifstream fs;
    fs.open(file_in);
    if (!fs.is_open())
    {
        ROS_ERROR("Cannot open file: %s", file_in.c_str());
        return {};
    }
    std_msgs::Float64MultiArray position_msg;
    position_msg.data.resize(11);
    std::vector<std::vector<double>> positions_vector;

    int i = 0;
    std::string lineStr;
    while (std::getline(fs, lineStr))
    {
        std::stringstream ss(lineStr);
        std::string item;
        std::vector<double> position_vector;

        while (std::getline(ss, item, ','))
        {
            // if(i == 0){
            //     i++;
            //     continue;
            // } //一样地
            //std::cout << item << std::endl;
            position_vector.push_back(std::stod(item));
        }
        //read add Pedestrian
        position_vector.push_back(-0.7 + i * 0.024 * 0.067);
        position_vector.push_back(-0.3 + i * -0.079 * 0.067);
        i++;
        position_vector.erase(position_vector.begin());
        positions_vector.push_back(position_vector);
        //ROS_INFO("Publishing: %s", position_msg.data[0].c_str());
        //ROS_INFO("Publishing: %s", position_msg.data[1].c_str());
    }

    // for (std::vector<std::vector<double>>::iterator iter = positions_vector.begin(); iter!=positions_vector.end();iter++)
    // {
    //     std::cout << (*iter)[0] << "," << (*iter)[1] << "," << (*iter)[2] << "," << (*iter)[3] << ","
    //         << (*iter)[4] << "," << (*iter)[5] << "," << (*iter)[6] << "," << (*iter)[7] << "," << (*iter)[8]
    //         << "," << (*iter)[9] << "," << (*iter)[10] << std::endl;
    // }

    ros::Publisher position_pub = nh.advertise<std_msgs::Float64MultiArray>("csv_test_pos", 1);
    ros::Rate loop_rate(20);

    //行人的起始位置和速度，倒数第二个 //0.9,-5.0

    int start = 0;

    while (ros::ok())
    {

        position_msg.data = positions_vector[start]; //放入publish的msg中
        if(start < positions_vector.size() - 1)
            start++;

        position_pub.publish(position_msg); //发布

        ros::spinOnce(); //调用对应的回调函数
        loop_rate.sleep(); //保证rate
    }

    return 0;
}