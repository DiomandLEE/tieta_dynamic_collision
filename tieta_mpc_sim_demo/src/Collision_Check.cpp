#include "tieta_mpc_sim_demo/Collision_Check.h"
Collision_Check::Collision_Check(/* args */)
{
}

Collision_Check::~Collision_Check()
{
}

std::vector<Dvector> Collision_Check::get_sphere_position(){return collision_points;}

void Collision_Check::calculate_sphere_position(const std::vector<double> currentJointPositions){
    //从jointstate转为JntArray
    // std::cout << "COllison_Check::DEbug 1" << std::endl;
    // std::cout << "currentJointPositions.size() = " << currentJointPositions.size() << std::endl;
    // std::cout << "joint_positions_ee_tool.data.size() = " << joint_positions_ee_tool.data.size() << std::endl;
    // std::cout << "joint_positions_shoulder.data.size() = " << joint_positions_shoulder.data.size() << std::endl;
    // std::cout << "joint_positions_elbow.data.size() = " << joint_positions_elbow.data.size() << std::endl;
    // std::cout << "joint_positions_wrist.data.size() = " << joint_positions_wrist.data.size() << std::endl;
    // std::cout << "joint_positions_gripper.data.size() = " << joint_positions_gripper.data.size() << std::endl;
    for(unsigned int i = 0; i < joint_positions_ee_tool.data.size(); i++)
    {
        if(i < joint_positions_shoulder.data.size())
        {
            joint_positions_shoulder.data[i] = currentJointPositions[i];
            joint_positions_elbow.data[i] = currentJointPositions[i];
            joint_positions_wrist.data[i] = currentJointPositions[i];
            joint_positions_gripper.data[i] = currentJointPositions[i];
            //
            joint_positions_ee_tool.data[i] = currentJointPositions[i];
        }
        if(i >= joint_positions_shoulder.data.size() && i < joint_positions_elbow.data.size())
        {
            joint_positions_elbow.data[i] = currentJointPositions[i];
            joint_positions_wrist.data[i] = currentJointPositions[i];
            joint_positions_gripper.data[i] = currentJointPositions[i];
            //
            joint_positions_ee_tool.data[i] = currentJointPositions[i];
        }
        if(i >= joint_positions_elbow.data.size() && i < joint_positions_wrist.data.size())
        {
            joint_positions_wrist.data[i] = currentJointPositions[i];
            joint_positions_gripper.data[i] = currentJointPositions[i];
            //
            joint_positions_ee_tool.data[i] = currentJointPositions[i];
        }
        if(i >= joint_positions_wrist.data.size() && i < joint_positions_gripper.data.size())
        {
            joint_positions_gripper.data[i] = currentJointPositions[i];
            //
            joint_positions_ee_tool.data[i] = currentJointPositions[i];
        }
        if(i >= joint_positions_gripper.data.size() && i < joint_positions_ee_tool.data.size())
            joint_positions_ee_tool.data[i] = currentJointPositions[i];

    }
    ////std::cout << "COllison_Check::DEbug 2" << std::endl;
    //计算正运动学，并获取位置xyz结果
    KDL::ChainFkSolverPos_recursive fk_solver_shoulder(kdl_chain_shoulder);
    KDL::ChainFkSolverPos_recursive fk_solver_elbow(kdl_chain_elbow);
    KDL::ChainFkSolverPos_recursive fk_solver_wrist(kdl_chain_wrist);
    KDL::ChainFkSolverPos_recursive fk_solver_gripper(kdl_chain_gripper);
    KDL::ChainFkSolverPos_recursive fk_solver_ee_tool(kdl_chain_ee_tool);

    fk_solver_shoulder.JntToCart(joint_positions_shoulder, frame_shoulder);
    fk_solver_elbow.JntToCart(joint_positions_elbow, frame_elbow);
    fk_solver_wrist.JntToCart(joint_positions_wrist, frame_wrist);
    fk_solver_gripper.JntToCart(joint_positions_gripper, frame_gripper);
    fk_solver_ee_tool.JntToCart(joint_positions_ee_tool, frame_ee_tool);

    size_t var_size = 3;
    Dvector shoulder_sphere_position(var_size);
    Dvector elbow_sphere_position(var_size);
    Dvector wrist_sphere_position(var_size);
    Dvector gripper_sphere_position(var_size);
    Dvector ee_tool_sphere_position(var_size);
    Dvector ee_tool_sphere_orientation(var_size);
    std::vector<double> ee_tool_sphere_orientation_temp(var_size);

    shoulder_sphere_position[0] = frame_shoulder.p.x();
    shoulder_sphere_position[1] = frame_shoulder.p.y();
    shoulder_sphere_position[2] = frame_shoulder.p.z();
    elbow_sphere_position[0] = frame_elbow.p.x();
    elbow_sphere_position[1] = frame_elbow.p.y();
    elbow_sphere_position[2] = frame_elbow.p.z();
    wrist_sphere_position[0] = frame_wrist.p.x();
    wrist_sphere_position[1] = frame_wrist.p.y();
    wrist_sphere_position[2] = frame_wrist.p.z();
    gripper_sphere_position[0] = frame_gripper.p.x();
    gripper_sphere_position[1] = frame_gripper.p.y();
    gripper_sphere_position[2] = frame_gripper.p.z();

    ee_tool_sphere_position[0] = frame_ee_tool.p.x();
    ee_tool_sphere_position[1] = frame_ee_tool.p.y();
    ee_tool_sphere_position[2] = frame_ee_tool.p.z();
    frame_ee_tool.M.GetRPY(ee_tool_sphere_orientation_temp[0], ee_tool_sphere_orientation_temp[1], ee_tool_sphere_orientation_temp[2]);
    ee_tool_sphere_orientation[0] = ee_tool_sphere_orientation_temp[0];
    ee_tool_sphere_orientation[1] = ee_tool_sphere_orientation_temp[1];
    ee_tool_sphere_orientation[2] = ee_tool_sphere_orientation_temp[2];
    ////std::cout << "COllison_Check::DEbug 3" << std::endl;

    //计算底盘的膨胀球体
    R_robot_world << std::cos(currentJointPositions[2]), -std::sin(currentJointPositions[2]), 0,
            std::sin(currentJointPositions[2]), std::cos(currentJointPositions[2]), 0,
            0, 0, 1;
    t_robot_world << currentJointPositions[0], currentJointPositions[1], 0.0;

    sphere_lf_world = R_robot_world * sphere_lf_robot + t_robot_world;
    sphere_rf_world = R_robot_world * sphere_rf_robot + t_robot_world;
    sphere_lr_world = R_robot_world * sphere_lr_robot + t_robot_world;
    sphere_rr_world = R_robot_world * sphere_rr_robot + t_robot_world;

    Dvector vars_sphere_lf(var_size);
    Dvector vars_sphere_rf(var_size);
    Dvector vars_sphere_lr(var_size);
    Dvector vars_sphere_rr(var_size);

    //debug 问题就是出现在时限没有指定上面四个vars的size，才会导致cppAD报错
    vars_sphere_lf[0] = sphere_lf_world[0];
    vars_sphere_lf[1] = sphere_lf_world[1];
    ////std::cout << "COllison_Check::DEbug 4" << std::endl;
    vars_sphere_lf[2] = sphere_lf_world[2];
    vars_sphere_rf[0] = sphere_rf_world[0];
    vars_sphere_rf[1] = sphere_rf_world[1];
    vars_sphere_rf[2] = sphere_rf_world[2];
    vars_sphere_lr[0] = sphere_lr_world[0];
    vars_sphere_lr[1] = sphere_lr_world[1];
    vars_sphere_lr[2] = sphere_lr_world[2];
    vars_sphere_rr[0] = sphere_rr_world[0];
    vars_sphere_rr[1] = sphere_rr_world[1];
    vars_sphere_rr[2] = sphere_rr_world[2];
    ///std::cout << "COllison_Check::DEbug 1" << std::endl;

    collision_points.push_back(vars_sphere_lf);
    collision_points.push_back(vars_sphere_rf);
    collision_points.push_back(vars_sphere_rf);
    collision_points.push_back(vars_sphere_rr);

    //! //计算两个位置之间的距离
    // double distance = (shoulder_sphere_position - elbow_sphere_position).norm();
    collision_points.push_back(shoulder_sphere_position);
    collision_points.push_back(elbow_sphere_position);
    collision_points.push_back(wrist_sphere_position);
    collision_points.push_back(gripper_sphere_position);
    collision_points.push_back(ee_tool_sphere_position);
    collision_points.push_back(ee_tool_sphere_orientation);

    fk_solver_shoulder.~ChainFkSolverPos_recursive();
    fk_solver_elbow.~ChainFkSolverPos_recursive();
    fk_solver_wrist.~ChainFkSolverPos_recursive();
    fk_solver_gripper.~ChainFkSolverPos_recursive();
    fk_solver_ee_tool.~ChainFkSolverPos_recursive();
}