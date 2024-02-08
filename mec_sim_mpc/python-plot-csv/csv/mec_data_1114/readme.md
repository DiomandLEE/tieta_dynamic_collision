11:12:59:

# Parameters for control loop
pub_twist_cmd: true
debug_info: true
delay_mode: false
#waypoints_dist: -1.0 # unit: m, set < 0 means computed by node

path_length: 3.0 # unit: m
goal_radius: 0.5 # unit: m
controller_freq: 10
pid_freq: 10

# Parameter for MPC solver
mpc_steps: 10.0

mpc_w_distx: 150000.0
mpc_w_disty: 15000.0
mpc_w_etheta: 10000.0

mpc_w_vel: 200.0
mpc_w_angvel: 200.0

mpc_w_angacc: 200.0
mpc_w_acc: 200.0

mpc_max_vel: 0.2 # unit: m/s
max_speed: 0.1

mpc_max_angvel: 0.5 #先看一遍CSV,theta之间的最大的差是0.03,最大的角速度是就是0.3
mpc_max_acc: 0.5 # Maximal accel

mpc_bound_value: 1.0e3 # Bound value for other variables
angel_upper_bound: M_PI
angel_lower_bound: -M_PI

#frame name
map_frame: "/vicon/world"
car_frame: "vicon/mec_kinova/mec_kinova"

#tolerence
tolerence_position: 0.005
tolerence_angle: 0.02

#kinova reach init flag , for invidual debug
kinova_reach_init_flag: true
mec_reach_init_flag: false
#csvfile_path
#csvfile_path: "../data/csv_data/csv_data.csv"
savefolder_path: "/home/tongbot/tongbot_mpc_ws/src/mec_speed_mpc/record_csv"
save_Debugfolder_path: "/home/tongbot/tongbot_mpc_ws/src/mec_speed_mpc/debug_csv"
#save_Debugfile_path_Class_MPC: "/home/tongbot/tongbot_mpc_ws/src/mec_speed_mpc/debug_csv/debug_data_test2.csv"
#save_Debugfile_path_Class_FG_eval: "/home/tongbot/tongbot_mpc_ws/src/mec_speed_mpc/debug_csv/debug_data_test3.csv"

#pid参数
#x
x_kp: 0.2
x_ki: 0.01
x_kd: 0.005
#y
y_kp: 0.2
y_kd: 0.01
y_ki: 0.005
#yaw
yaw_kp: 0.1
yaw_ki: 0.02
yaw_kd: 0.01



11:15:37

# Parameters for control loop
pub_twist_cmd: true
debug_info: true
delay_mode: true
#waypoints_dist: -1.0 # unit: m, set < 0 means computed by node

path_length: 3.0 # unit: m
goal_radius: 0.5 # unit: m
controller_freq: 10
pid_freq: 10

# Parameter for MPC solver
mpc_steps: 10.0

mpc_w_distx: 150000.0
mpc_w_disty: 15000.0
mpc_w_etheta: 10000.0

mpc_w_vel: 200.0
mpc_w_angvel: 200.0

mpc_w_angacc: 200.0
mpc_w_acc: 200.0

mpc_max_vel: 0.2 # unit: m/s
max_speed: 0.1

mpc_max_angvel: 0.5 #先看一遍CSV,theta之间的最大的差是0.03,最大的角速度是就是0.3
mpc_max_acc: 0.5 # Maximal accel

mpc_bound_value: 1.0e3 # Bound value for other variables
angel_upper_bound: M_PI
angel_lower_bound: -M_PI

#frame name
map_frame: "/vicon/world"
car_frame: "vicon/mec_kinova/mec_kinova"

#tolerence
tolerence_position: 0.005
tolerence_angle: 0.02

#kinova reach init flag , for invidual debug
kinova_reach_init_flag: true
mec_reach_init_flag: false
#csvfile_path
#csvfile_path: "../data/csv_data/csv_data.csv"
savefolder_path: "/home/tongbot/tongbot_mpc_ws/src/mec_speed_mpc/record_csv"
save_Debugfolder_path: "/home/tongbot/tongbot_mpc_ws/src/mec_speed_mpc/debug_csv"
#save_Debugfile_path_Class_MPC: "/home/tongbot/tongbot_mpc_ws/src/mec_speed_mpc/debug_csv/debug_data_test2.csv"
#save_Debugfile_path_Class_FG_eval: "/home/tongbot/tongbot_mpc_ws/src/mec_speed_mpc/debug_csv/debug_data_test3.csv"

#pid参数
#x
x_kp: 0.2
x_ki: 0.01
x_kd: 0.005
#y
y_kp: 0.2
y_kd: 0.01
y_ki: 0.005
#yaw
yaw_kp: 0.1
yaw_ki: 0.02
yaw_kd: 0.01




