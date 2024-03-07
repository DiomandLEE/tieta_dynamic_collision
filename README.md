进入工作空间文件夹
```yaml
cd VKConTieta_ws
source devel/setup.bash
```
1、运行Rviz加载tieta以及env模型
```yaml
roslaunch urdf_tutorial display.launch model:=/home/diamondlee/VKConTieta_ws/src/urdf_description/robot_description/ridgeback_dual_arm_description/urdf/vkc_big_task.urdf.xacro
```
2、通过自己的节点发布Joint State，关闭display.launch
```yaml
rosnode kill /joint_state_publisher
```
3.检查traj.csv是否合理：
```yaml
rosrun tieta_mpc_sim_demo csv_test_pub_node \ rosurn tieta_mpc_sim_demo csv_test_sub_node
```
4.运行自己编写的接收mpc_node的速度topic的仿真：
```yaml
roslaunch tieta_mpc_sim_demo rviz_sim.launch
```
5.解析parse traj.csv并发布
```yaml
roslaunch JointTrajPub csv_pub_anglesList.launch
```
6.运行mpc_node
```yaml
source devel/setup.bash
cd LOGS
roslaunch tieta_mpc_sim_demo mpc_run.launch 2>&1 | tee mpc_run_日期_时间.log
```