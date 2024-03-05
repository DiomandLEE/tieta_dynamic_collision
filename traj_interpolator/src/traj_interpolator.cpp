// #include <fstream>
// #include <iostream>
// #include <string>
// #include <vector>
// #include <ros/ros.h>
// #include <robot_trajectory/robot_trajectory.h>
// #include <trajectory_processing/iterative_time_parameterization.h>

// // 读取CSV文件
// std::vector<std::vector<double>> readCsvFile(const std::string& file_path) {
//     std::vector<std::vector<double>> trajectory;
//     std::ifstream file(file_path);
//     if (file.is_open()) {
//         std::string line;
//         while (getline(file, line)) {
//             std::istringstream iss(line);
//             std::vector<double> row;
//             std::string value;
//             while (getline(iss, value, ',')) {
//                 row.push_back(std::stod(value));
//             }
//             trajectory.push_back(row);
//         }
//         file.close();
//     }
//     return trajectory;
// }

// // 插值轨迹
// moveit_msgs::RobotTrajectory interpolateTrajectory(const moveit_msgs::RobotTrajectory& trajectory,
//                                                     double timestep, double max_velocity) {
//     moveit_msgs::RobotTrajectory interpolated_traj;
//     interpolated_traj.joint_trajectory.joint_names = trajectory.joint_trajectory.joint_names;

//     trajectory_msgs::JointTrajectoryPoint last_point = trajectory.joint_trajectory.points[0];
//     interpolated_traj.joint_trajectory.points.push_back(last_point);

//     for (int i = 1; i < trajectory.joint_trajectory.points.size(); i++) {
//         double dt = (trajectory.joint_trajectory.points[i].time_from_start - last_point.time_from_start).toSec();
//         int num_steps = static_cast<int>(dt / timestep);

//         for (int j = 1; j <= num_steps; j++) {
//             trajectory_msgs::JointTrajectoryPoint new_point;
//             new_point.time_from_start = last_point.time_from_start + ros::Duration(j * timestep);

//             for (int k = 0; k < interpolated_traj.joint_trajectory.joint_names.size(); k++) {
//                 double last_joint_value = last_point.positions[k];
//                 double current_joint_value = trajectory.joint_trajectory.points[i].positions[k];
//                 double max_increment = max_velocity * timestep;

//                 double diff = current_joint_value - last_joint_value;
//                 double increment = std::min(max_increment, std::abs(diff));
//                 double new_joint_value = last_joint_value + increment * (diff / std::abs(diff));

//                 new_point.positions.push_back(new_joint_value);
//                 new_point.velocities.push_back((new_joint_value - last_joint_value) / timestep);
//             }

//             interpolated_traj.joint_trajectory.points.push_back(new_point);
//         }
//         last_point = trajectory.joint_trajectory.points[i];
//     }

//     return interpolated_traj;
// }

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "trajectory_interpolator");
//     ros::NodeHandle nh;

//     // 读取CSV文件
//     std::string csv_file = "/path/to/your/csv/file.csv";
//     std::vector<std::vector<double>> raw_trajectory = readCsvFile(csv_file);

//     // 创建MoveIt的轨迹对象
//     moveit_msgs::RobotTrajectory trajectory;
//     // 填充关节名称
//     trajectory.joint_trajectory.joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

//     // 填充轨迹点
//     for (const auto& row : raw_trajectory) {
//         trajectory_msgs::JointTrajectoryPoint point;
//         point.positions = row;
//         trajectory.joint_trajectory.points.push_back(point);
//     }

//     // 插值轨迹
//     moveit_msgs::RobotTrajectory interpolated_traj = interpolateTrajectory(trajectory, 0.01, 0.005);

//     // 时间参数化
//     trajectory_processing::IterativeParabolicTimeParameterization time_parameterization;
//     bool success = time_parameterization.computeTimeStamps(interpolated_traj);

//     if (success) {
//         // 打印插值后的轨迹
//         ROS_INFO_STREAM("Interpolated trajectory: " << interpolated_traj);
//     } else {
//         ROS_ERROR("Failed to compute time stamps for the interpolated trajectory.");
//     }

//     return 0;
// }