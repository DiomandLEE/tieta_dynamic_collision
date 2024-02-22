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
    for(unsigned int i = 0; i < joint_positions_gripper.data.size(); i++)
    {
        if(i < joint_positions_shoulder.data.size())
        {
            joint_positions_shoulder.data[i] = currentJointPositions[i];
            joint_positions_elbow.data[i] = currentJointPositions[i];
            joint_positions_wrist.data[i] = currentJointPositions[i];
            joint_positions_gripper.data[i] = currentJointPositions[i];
        }
        if(i <= joint_positions_shoulder.data.size() && i < joint_positions_elbow.data.size())
        {
            joint_positions_elbow.data[i] = currentJointPositions[i];
            joint_positions_wrist.data[i] = currentJointPositions[i];
            joint_positions_gripper.data[i] = currentJointPositions[i];
        }
        if(i <= joint_positions_elbow.data.size() && i < joint_positions_wrist.data.size())
        {
            joint_positions_wrist.data[i] = currentJointPositions[i];
            joint_positions_gripper.data[i] = currentJointPositions[i];
        }
        if(i <= joint_positions_wrist.data.size() && i < joint_positions_gripper.data.size())
            joint_positions_gripper.data[i] = currentJointPositions[i];
    }
    //计算正运动学，并获取位置xyz结果
    KDL::ChainFkSolverPos_recursive fk_solver_shoulder(kdl_chain_shoulder);
    KDL::ChainFkSolverPos_recursive fk_solver_elbow(kdl_chain_elbow);
    KDL::ChainFkSolverPos_recursive fk_solver_wrist(kdl_chain_wrist);
    KDL::ChainFkSolverPos_recursive fk_solver_gripper(kdl_chain_gripper);

    fk_solver_shoulder.JntToCart(joint_positions_shoulder, frame_shoulder);
    fk_solver_elbow.JntToCart(joint_positions_elbow, frame_elbow);
    fk_solver_wrist.JntToCart(joint_positions_wrist, frame_wrist);
    fk_solver_gripper.JntToCart(joint_positions_gripper, frame_gripper);

    size_t var_size = 3;
    Dvector shoulder_sphere_position(var_size);
    Dvector elbow_sphere_position(var_size);
    Dvector wrist_sphere_position(var_size);
    Dvector gripper_sphere_position(var_size);

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

    //计算底盘的膨胀球体
    R_robot_world << std::cos(currentJointPositions[2]), -std::sin(currentJointPositions[2]), 0,
            std::sin(currentJointPositions[2]), std::cos(currentJointPositions[2]), 0,
            0, 0, 1;
    t_robot_world << currentJointPositions[0], currentJointPositions[1], 0.0;

    sphere_lf_world = R_robot_world * sphere_lf_robot + t_robot_world;
    sphere_rf_world = R_robot_world * sphere_rf_robot + t_robot_world;
    sphere_lr_world = R_robot_world * sphere_lr_robot + t_robot_world;
    sphere_rr_world = R_robot_world * sphere_rr_robot + t_robot_world;

    Dvector vars_sphere_lf;
    Dvector vars_sphere_rf;
    Dvector vars_sphere_lr;
    Dvector vars_sphere_rr;

    vars_sphere_lf[0] = sphere_lf_world[0];
    vars_sphere_lf[1] = sphere_lf_world[1];
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

    fk_solver_shoulder.~ChainFkSolverPos_recursive();
    fk_solver_elbow.~ChainFkSolverPos_recursive();
    fk_solver_wrist.~ChainFkSolverPos_recursive();
    fk_solver_gripper.~ChainFkSolverPos_recursive();
}