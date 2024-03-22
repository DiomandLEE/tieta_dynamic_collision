import matplotlib.pyplot as plt
import pandas as pd

# 读取CSV文件
distance_csv = pd.read_csv("/home/diamondlee/VKConTieta_ws/src/tieta_mpc_sim_demo/distance_results/left_down2right_up_0319_115105_closet.csv")

# 创建时间列，以0.025的时间间隔
time_column = [i * 0.025 for i in range(len(distance_csv))]

time_collision = [i * 1.25 for i in range(len(distance_csv))]

# 设置子图布局
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 9.7), gridspec_kw={'height_ratios': [2, 3]})  # 调整子图尺寸

# 定义每条曲线的颜色
colors = ['blue', 'orange', 'cyan', 'pink', 'purple']

# 绘制曲线
ax1.plot(time_column, distance_csv.iloc[:, 0], label='Base', color=colors[0])

ax1.plot(time_column, [1.0] * len(time_column), '--', color='green', label='Soft Constraint',linewidth=2.5)
#ax1.axhline(y=0.06, color='black', linestyle='--')
# ax1.text(0,1.25,'1.25',fontsize=10.5,color='black',ha='right') #base
# 添加碰撞线
ax1.plot(time_collision, [0.65] * len(time_collision), 'o-', color='red', label='Collision',markerfacecolor='none', linewidth=2.5)
#ax1.axhline(y=0.06, color='black', linestyle='--')
# ax1.text(0,0.9,'0.9',fontsize=10.5,color='black',ha='right') #base

# 添加图例和标签
# ax1.legend(fontsize="large")
# ax1.xlabel("Time (s)")
# ax1.ylabel("Distance (m)")
# ax1.title('Base Distance from Pedestrian Over Time')

# 设置y轴刻度
# #closet
# ax1.yticks([0, 0.5, 1, 1.5, 2, 2.5, 3, 3.5, 4])
# ax1.ylim(0, 3.5)
ax1.set_yticks([0.5, 1, 1.5, 2, 2.5, 3, 3.5])
ax1.set_ylim(0.5, 3.0)
ax1.set_xlim(0, 0.025 * len(distance_csv))

ax1.legend(loc="upper right",bbox_to_anchor=(.99, 1.0))
ax1.grid(True)
ax1.set_xlabel('Time (s)',fontweight='bold')
ax1.set_ylabel('Distance (m)',fontweight='bold')
ax1.set_title('Base Distance from Closet Over Time',fontweight='bold')



ax2.plot(time_column, distance_csv.iloc[:, 1], label='Shoulder', color=colors[1])
ax2.plot(time_column, distance_csv.iloc[:, 2], label='Elbow', color=colors[2])
ax2.plot(time_column, distance_csv.iloc[:, 3], label='Wrist', color=colors[3])
ax2.plot(time_column, distance_csv.iloc[:, 4], '-.',label='Gripper', color=colors[4]) #closet

ax2.plot(time_column, [0.2] * len(time_column), '--', color='green', label='Soft Constraint',linewidth=2.5)
#ax2.axhline(y=0.06, color='black', linestyle='--')
# ax2.text(0,0.8,'0.8',fontsize=10.5,color='black',ha='right') #base
# 添加碰撞线
ax2.plot(time_collision, [0.06] * len(time_collision), 'o-', color='red', label='Collision',markerfacecolor='none', linewidth=2.5)
#ax2.axhline(y=0.06, color='black', linestyle='--')
# ax2.text(0,0.4,'0.4',fontsize=10.5,color='black',ha='right') #base

ax2.set_yticks([0, 0.5, 1, 1.5, 2, 2.5, 3])
ax2.set_ylim(0, 3)
ax2.set_xlim(0, 0.025 * len(distance_csv))

ax2.legend(loc="upper right",bbox_to_anchor=(.99, 1.0))
ax2.grid(True)
ax2.set_xlabel('Time (s)',fontweight='bold')
ax2.set_ylabel('Distance (m)',fontweight='bold')
ax2.set_title('Arm Distance from Closet Over Time',fontweight='bold')

plt.xticks(fontsize=10)
plt.yticks(fontsize=10)

# 调整布局，防止标签被截断
plt.tight_layout()

# 显示图形
plt.show()