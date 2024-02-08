import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import make_interp_spline

# 从CSV文件加载数据

# 从第二个CSV文件加载数据
#data1 = pd.read_csv('csv/mec_data_1114/plot_20231114_111259/exe_traj.csv')
#data1 = pd.read_csv('csv/mec_data_1114_sim/sim_plot_20231114_162048/exe_traj.csv')
data1 = pd.read_csv('/home/diamondlee/mec-arm-ws/mec_mpc_sim/src/mec_speed_mpc/record_csv/sim_plot_20231116_171802/exe_traj.csv')

# # 提取第三列的数据并保留小数点后三位
# data2.iloc[1:, 2] = data2.iloc[1:, 2].apply(lambda x: round(x, 4))

# 提取第六列，第二行开始后的数据
y1 = data1.iloc[:, 5].apply(lambda x: round(x, 4))  # 前面指的是第二行，2是指的是第三列
#print(len(y1))

# 创建横轴数据，间隔为1
x1 = range(3, len(y1) + 3)

#获得prediction_traj
#data3 = pd.read_csv('csv/mec_data/record_csv/plot_20231111_182947/prediction_traj.csv')
#data2 = pd.read_csv('csv/mec_data_1114/plot_20231114_111259/prediction_traj.csv')
#data2 = pd.read_csv('csv/mec_data_1114_sim/sim_plot_20231114_162048/prediction_traj.csv')
data2 = pd.read_csv('/home/diamondlee/mec-arm-ws/mec_mpc_sim/src/mec_speed_mpc/record_csv/sim_plot_20231116_171802/prediction_traj.csv')

# 提取第一列，第二行开始后的数据
y2 = data1.iloc[:, 8].apply(lambda x: round(x, 4))  # 前面指的是第二行，2是指的是第三列

#####################################PREDICT###########################################3
# x3_start = 3
# mpc_horizon = 9 - 1
# map_vlocitys = []
# # 画出每一段预测的traj曲线
# for i in range(21, len(data2), 30):
#     map_vlocitys.append(y2[i])
# print(len(map_vlocitys))
#############################################################################################
# 添加网格

# 创建横轴数据，间隔为1
#x2 = range(1, len(map_vlocitys) + 1) #!delay:1 non:2
x2 = range(1, len(y2) + 1) #!delay:1 non:2

# 创建折线图
plt.plot(x2, y2, label='EXE', marker='o', markersize=10, markerfacecolor='red', linewidth=5)
plt.plot(x1, y1, label='PREDICT', marker='s', markersize=10, markerfacecolor='blue', linewidth=5)

# 添加标题和标签
plt.title('SPEED_EXE_PREDICT')
plt.xlabel('loop_cnt')
plt.ylabel('map_speed_X')
# 调整坐标轴刻度间隔
plt.xticks(range(1, max(len(x1),len(x2)) + 1), rotation=45)  # 设置x轴刻度间隔为1，可根据需求调整旋转角度
# y_ticks = np.arange(-0.03,
#                     0.23, 0.01)
# plt.yticks(y_ticks, [f'{i:.3f}' for i in y_ticks])
plt.yticks(y1, fontsize=12)

# 在每个数据点上标出数值
for i, j in zip(x1, y1):
    plt.text(i, j, str(j), ha='right', va='bottom', fontsize=15, rotation=45)
# for i, j in zip(x2, map_vlocitys):
#     plt.text(i, j, str(j), ha='right', va='bottom', fontsize=15, rotation=0)
for i, j in zip(x2, y2):
    plt.text(i, j, str(j), ha='right', va='bottom', fontsize=15, rotation=0)

plt.grid(True)

# 显示图例
plt.legend()

# 显示图表
plt.show()
