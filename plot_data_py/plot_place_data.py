# import matplotlib.pyplot as plt
# import numpy as np
# from pylab import rcParams

# # Sample data (Replace with your actual data)
# time = np.linspace(0, 16, 100)  # Simulating a time variable from 0 to 16 seconds
# data1 = np.sin(time) * 2 + time * 0.1  # Simulated data for Obstacle 1
# data2 = np.cos(time) * 2 + np.random.randn(100) * 0.2  # Simulated data for Obstacle 2
# data3 = np.sin(time + np.pi / 4) * 2 + time * 0.05  # Simulated data for Obstacle 3
# collision = (time > 5) & (time < 11)  # Collision periods
# print(collision)

# # Plotting the data
# plt.figure(figsize=(14, 7))
# plt.plot(time, data1, label='Obstacle 1', color='blue')
# plt.plot(time, data2, label='Obstacle 2', color='orange')
# plt.plot(time, data3, label='Obstacle 3', color='green')
# plt.plot(time, np.repeat(3, 100), label='Soft Constraint', color='red', linestyle='--')

# # Plotting collisions as points
# plt.scatter(time[collision], data1[collision], label='Collision', color='red', marker='o')

# # Customizing the plot
# plt.xlabel('Time (s)')
# plt.ylabel('Distance (m)')
# plt.title('Distance from Obstacles Over Time')
# plt.legend()
# plt.grid(True)

# # Display the plot
# plt.show()
import pandas as pd
import matplotlib.pyplot as plt

# 读取CSV文件
distance_csv1 = pd.read_csv("/home/diamondlee/VKConTieta_ws/src/open_door_tieta_mpc/distance_results/horizontal_left_handle.csv")
distance_csv2 = pd.read_csv("/home/diamondlee/VKConTieta_ws/src/open_door_tieta_mpc/distance_results/horizontal_right_handle.csv")
distance_csv3 = pd.read_csv("/home/diamondlee/VKConTieta_ws/src/open_door_tieta_mpc/distance_results/left_down2right_up_handle.csv")
distance_csv4 = pd.read_csv("/home/diamondlee/VKConTieta_ws/src/open_door_tieta_mpc/distance_results/left_up2right_down_handle.csv")
distance_csv5 = pd.read_csv("/home/diamondlee/VKConTieta_ws/src/open_door_tieta_mpc/distance_results/right_down2left_up_handle.csv")
distance_csv6 = pd.read_csv("/home/diamondlee/VKConTieta_ws/src/open_door_tieta_mpc/distance_results/right_up2left_down_handle.csv")

# 创建时间列，以0.025的时间间隔
time_column1 = [i * 0.025 for i in range(len(distance_csv1))]
time_column2 = [i * 0.025 for i in range(len(distance_csv2))]
time_column3 = [i * 0.025 for i in range(len(distance_csv3))]
time_column4 = [i * 0.025 for i in range(len(distance_csv4))]
time_column5 = [i * 0.025 for i in range(len(distance_csv5))]
time_column6 = [i * 0.025 for i in range(len(distance_csv6))]



time_collision = [i * 1.25 for i in range(len(distance_csv1))]

plt.figure(figsize=(8.6, 5.0))

# 定义每条曲线的颜色
colors = ['blue', 'orange', 'cyan', 'pink', 'purple' ,'brown']

# 绘制图形
plt.plot(time_column1, distance_csv1, label='Case1', color=colors[0],linewidth=2.3)
plt.plot(time_column2, distance_csv2, label='Case2', color=colors[1],linewidth=2.3)
plt.plot(time_column3, distance_csv3, label='Case3', color=colors[2],linewidth=2.3)
plt.plot(time_column4, distance_csv4, label='Case4', color=colors[3],linewidth=3)
# plt.plot(time_column, distance_csv4, '-.',distance_csv1label='Gripper', color=colors[4]) #closet
plt.plot(time_column5, distance_csv5, label='Case5', color=colors[4],linewidth=2.3)
plt.plot(time_column6, distance_csv6, label='Case6', color=colors[5],linewidth=2.3)

time_column = [i * 0.025 for i in range(min(len(distance_csv1), len(distance_csv2), len(distance_csv3), len(distance_csv4), len(distance_csv5), len(distance_csv6)))]


plt.plot(time_column, [1.15] * len(time_column), '--', color='green', label='Upper Limit',linewidth=2.5)
#plt.axhline(y=0.06, color='black', linestyle='--')
plt.text(0,1.15,'1.15',fontsize=10.5,color='black',ha='left') #base

plt.plot(time_column, [0.65] * len(time_column), '--', color='green', label='Lower Limit',linewidth=2.5)
#plt.axhline(y=0.06, color='black', linestyle='--')
plt.text(0,0.65,'0.65',fontsize=10.5,color='black',ha='left') #base



# 添加图例和标签
plt.legend(loc="upper right",bbox_to_anchor=(.99, 1.0),ncol=2)
plt.xlabel("Time (s)",fontweight='bold')
plt.ylabel("Distance (m)",fontweight='bold')
plt.title('Distance Shoulder from Handle Over Time',fontweight='bold')

# 设置y轴刻度
# #closet
# plt.yticks([0, 0.5, 1, 1.5, 2, 2.5, 3, 3.5, 4])
# plt.ylim(0, 3.5)
plt.yticks([0, 0.2, 0.4, 0.5,0.6, 0.8, 1.0, 1.2,1.5])
plt.ylim(0.5, 1.37)
plt.xlim(0, 0.025 * len(time_column))


plt.grid(True)
plt.tight_layout()
# 显示图形
plt.show()