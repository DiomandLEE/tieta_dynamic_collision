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
distance_csv = pd.read_csv("/home/diamondlee/VKConTieta_ws/src/tieta_mpc_sim_demo/distance_results/left_down2right_up_0319_115105_pedestrian.csv")

# 创建时间列，以0.025的时间间隔
time_column = [i * 0.025 for i in range(len(distance_csv))]

time_collision = [i * 1.25 for i in range(len(distance_csv))]

plt.figure(figsize=(8.6, 5.0))

# 定义每条曲线的颜色
colors = ['blue', 'orange', 'cyan', 'pink', 'purple']

# 绘制图形
plt.plot(time_column, distance_csv.iloc[:, 0], label='Base', color=colors[0])
plt.plot(time_column, distance_csv.iloc[:, 1], label='Shoulder', color=colors[1])
plt.plot(time_column, distance_csv.iloc[:, 2], label='Elbow', color=colors[2])
plt.plot(time_column, distance_csv.iloc[:, 3], label='Wrist', color=colors[3])
# plt.plot(time_column, distance_csv.iloc[:, 4], '-.',label='Gripper', color=colors[4]) #closet
plt.plot(time_column, distance_csv.iloc[:, 4], label='Gripper', color=colors[4])

# plt.plot(time_column, [1.25] * len(time_column), '--', color='green', label='Soft Constraint',linewidth=2.5)
# #plt.axhline(y=0.06, color='black', linestyle='--')
# plt.text(0,1.25,'1.25',fontsize=10.5,color='black',ha='right') #base
plt.plot(time_column, [0.8] * len(time_column), '--', color='green', label='Soft Constraint',linewidth=2.5)
#plt.axhline(y=0.06, color='black', linestyle='--')
plt.text(0,0.8,'0.8',fontsize=10.5,color='black',ha='right') #base
# 添加碰撞线
# plt.plot(time_collision, [0.9] * len(time_collision), 'o-', color='red', label='Collision',markerfacecolor='none', linewidth=2.5)
# #plt.axhline(y=0.06, color='black', linestyle='--')
# plt.text(0,0.9,'0.9',fontsize=10.5,color='black',ha='right') #base
plt.plot(time_collision, [0.4] * len(time_collision), 'o-', color='red', label='Collision',markerfacecolor='none', linewidth=2.5)
#plt.axhline(y=0.06, color='black', linestyle='--')
plt.text(0,0.4,'0.4',fontsize=10.5,color='black',ha='right') #base



# 添加图例和标签
plt.legend(fontsize="large")
plt.xlabel("Time (s)")
plt.ylabel("Distance (m)")
plt.title('Distance from Pedestrian Over Time')

# 设置y轴刻度
# #closet
# plt.yticks([0, 0.5, 1, 1.5, 2, 2.5, 3, 3.5, 4])
# plt.ylim(0, 3.5)
plt.yticks([0, 2, 4, 6, 8, 10, 12])
plt.ylim(0, 12)
plt.xlim(0, 0.025 * len(distance_csv))


plt.grid(True)
plt.tight_layout()
# 显示图形
plt.show()