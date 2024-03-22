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
    ros::init(argc, argv, "ikmpc_csv_pub_node");
    ros::NodeHandle nh;

    //!mpc_0312_170736 右上到左下 效果不好
    //!mpc_0319_160600 左下到右上 get
    //!mpc_0319_174902 水平从左往右 get
    //!mpc_0319_181831 水平从右往左 get
    //!mpc_0319_185509 左下到右上 效果不好
    //!mpc_0319_193553 右上到左下 get
    //!mpc_0321_114338 左上到右下 get
    //!mpc_0321_120158 右下到左上 get
    std::string file_in = "/home/diamondlee/VKConTieta_ws/src/open_door_tieta_mpc/positon_results/mpc_position_20240321_120158/mpc_all_joints_positions.csv";
    std::ifstream fs;
    fs.open(file_in);
    if (!fs.is_open())
    {
        ROS_ERROR("Cannot open file: %s", file_in.c_str());
        return {};
    }
    std_msgs::Float64MultiArray position_msg;
    position_msg.data.resize(12);
    std::vector<std::vector<double>> positions_vector_xy;

    std::string lineStr;
    while (std::getline(fs, lineStr))
    {
        std::stringstream ss(lineStr);
        std::string item;
        std::vector<double> position_vector;

        while (std::getline(ss, item, ','))
        {
            position_vector.push_back(std::stod(item));
            //i++;
        }
        positions_vector_xy.push_back(position_vector);
    }
    fs.close();

    //!ik_0314_205658
    //!ik_0319_162937
    //!ik_0319_175302
    //!ik_0319_182501
    //!ik_0319_185742
    //!ik_0319_195550
    //!ik_0321_114706
    //!ik_0321_120331
    std::string file_in1 = "/home/diamondlee/VKConTieta_ws/src/open_door_tieta_mpc/positon_results/ik_position_20240321_120331/ik_theta_URjoints_positions.csv";
    std::ifstream fs1;
    fs1.open(file_in1);
    if (!fs1.is_open())
    {
        ROS_ERROR("Cannot open file: %s", file_in1.c_str());
        return {};
    }
    std::vector<std::vector<double>> positions_vector_thetaUR;

    std::string lineStr1;
    while (std::getline(fs1, lineStr1))
    {
        std::stringstream ss(lineStr1);
        std::string item;
        std::vector<double> position_vector;

        while (std::getline(ss, item, ','))
        {
            position_vector.push_back(std::stod(item));
            //i++;
        }
        positions_vector_thetaUR.push_back(position_vector);
    }
    fs1.close();
    // for (std::vector<std::vector<double>>::iterator iter = positions_vector.begin(); iter!=positions_vector.end();iter++)
    // {
    //     std::cout << (*iter)[0] << "," << (*iter)[1] << "," << (*iter)[2] << "," << (*iter)[3] << ","
    //         << (*iter)[4] << "," << (*iter)[5] << "," << (*iter)[6] << "," << (*iter)[7] << "," << (*iter)[8] << std::endl;
    // }
    std::cout << "positions_vector_thetaUR.size() = " << positions_vector_thetaUR.size() << std::endl;
    std::cout << "positions_vector_xy.size() = " << positions_vector_xy.size() << std::endl;
    assert(positions_vector_thetaUR.size() == positions_vector_xy.size());

    ros::Publisher position_pub = nh.advertise<std_msgs::Float64MultiArray>("csv_ikmpc_pos", 1);
    ros::Rate loop_rate(10);

    int start = 0;

    while (ros::ok())
    {
        position_msg.data[0] = positions_vector_xy[start][0];
        position_msg.data[1] = positions_vector_xy[start][1];
        position_msg.data[2] = positions_vector_thetaUR[start][0];
        position_msg.data[3] = positions_vector_thetaUR[start][1];
        position_msg.data[4] = positions_vector_thetaUR[start][2];
        position_msg.data[5] = positions_vector_thetaUR[start][3];
        position_msg.data[6] = positions_vector_thetaUR[start][4];
        position_msg.data[7] = positions_vector_thetaUR[start][5];
        position_msg.data[8] = positions_vector_thetaUR[start][6];
        position_msg.data[9] = positions_vector_xy[start][2];
        position_msg.data[10] = positions_vector_xy[start][3];
        position_msg.data[11] = positions_vector_xy[start][4];
        //position_msg.data = positions_vector_xy[start]; //放入publish的msg中
        if(start < positions_vector_xy.size() - 1)
            start++;

        position_pub.publish(position_msg); //发布

        ros::spinOnce(); //调用对应的回调函数
        loop_rate.sleep(); //保证rate
    }

    return 0;
}