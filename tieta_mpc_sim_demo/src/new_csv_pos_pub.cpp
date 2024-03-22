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

    std::string file_in = "/home/diamondlee/VKConTieta_ws/src/tieta_mpc_sim_demo/position_results/mpc_position_20240318_182603/mpc_all_joints_positions.csv";
    std::ifstream fs;
    fs.open(file_in);
    if (!fs.is_open())
    {
        ROS_ERROR("Cannot open file: %s", file_in.c_str());
        return {};
    }
    std_msgs::Float64MultiArray position_msg;
    position_msg.data.resize(9);
    std::vector<std::vector<double>> positions_vector;

    std::string lineStr;
    while (std::getline(fs, lineStr))
    {
        std::stringstream ss(lineStr);
        std::string item;
        std::vector<double> position_vector;
        int i = 0; //把时间戳去掉
        while (std::getline(ss, item, ','))
        {
            // if(i == 0){
            //     i++;
            //     continue;
            // } //一样地
            //std::cout << item << std::endl;
            position_vector.push_back(std::stod(item));
            i++;
        }
        //position_vector.erase(position_vector.begin());
        positions_vector.push_back(position_vector);
        //ROS_INFO("Publishing: %s", position_msg.data[0].c_str());
        //ROS_INFO("Publishing: %s", position_msg.data[1].c_str());
    }
    //!0318_173803
    // int tuned_ = 1;
    // for (int i = (positions_vector.size() - 130); i < positions_vector.size(); i++)
    // {
    //     positions_vector[i][0] = positions_vector[i][0] - tuned_ * 0.00033385;
    //     positions_vector[i][1] = positions_vector[i][1] - tuned_ * 0.002;
    //     positions_vector[i][2] = positions_vector[i][2] + tuned_ * 0.0008615;
    //     tuned_++;
    // }
    //!0318_182603 都是从左到右，水平 //选这个，上一个离handle太远了
    int tuned_ = 1;
    for (int i = (positions_vector.size() - 300); i < positions_vector.size(); i++)
    {
        positions_vector[i][0] = positions_vector[i][0] - tuned_ * 0.00025;
        positions_vector[i][1] = positions_vector[i][1] - tuned_ * 0.00040333;
        //positions_vector[i][2] = positions_vector[i][2] + tuned_ * 0.0008615;
        tuned_++;
    }
    //!0318_192922 //水平，从右到左
    //无需修改
    //但是发布频率从30提高到40
    //!0318_195452 //左上到右下
    //无需修改
    //但是发布频率从30提高到35
    //!0319_103832 //右下到左上
    //但是发布频率回到30
    // int tuned_ = 1;
    // for (int i = (positions_vector.size() - 300); i < positions_vector.size(); i++)
    // {
    //     //positions_vector[i][0] = positions_vector[i][0] - tuned_ * 0.00025;
    //     positions_vector[i][1] = positions_vector[i][1] - tuned_ * 0.00012668;
    //     //positions_vector[i][2] = positions_vector[i][2] + tuned_ * 0.0008615;
    //     tuned_++;
    // }
    //!0319_105509 //右上到左下
    // int tuned_ = 1;
    // for (int i = (positions_vector.size() - 300); i < positions_vector.size(); i++)
    // {
    //     //positions_vector[i][0] = positions_vector[i][0] - tuned_ * 0.00025;
    //     positions_vector[i][1] = positions_vector[i][1] - tuned_ * 0.000028;
    //     //positions_vector[i][2] = positions_vector[i][2] + tuned_ * 0.0008615;
    //     tuned_++;
    // }
    //频率由30调到为25
    //!0319_115105 //左下到右上
    // int tuned_ = 1;
    // for (int i = (positions_vector.size() - 500); i < positions_vector.size(); i++)
    // {
    //     //positions_vector[i][0] = positions_vector[i][0] - tuned_ * 0.00025;
    //     positions_vector[i][1] = positions_vector[i][1] - tuned_ * 0.00006828;
    //     //positions_vector[i][2] = positions_vector[i][2] + tuned_ * 0.0008615;
    //     tuned_++;
    // }
    //频率由30调到为20

    // for (std::vector<std::vector<double>>::iterator iter = positions_vector.begin(); iter!=positions_vector.end();iter++)
    // {
    //     std::cout << (*iter)[0] << "," << (*iter)[1] << "," << (*iter)[2] << "," << (*iter)[3] << ","
    //         << (*iter)[4] << "," << (*iter)[5] << "," << (*iter)[6] << "," << (*iter)[7] << "," << (*iter)[8] << std::endl;
    // }

    ros::Publisher position_pub = nh.advertise<std_msgs::Float64MultiArray>("csv_test_pos", 1);
    ros::Rate loop_rate(20);

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