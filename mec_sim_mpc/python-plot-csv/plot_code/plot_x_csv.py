import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import make_interp_spline

# 从CSV文件加载数据
data1 = pd.read_csv('/home/diamondlee/mec-arm-ws/mec_mpc_sim/src/anglesPub/csv/retimed_traj.csv')

# 提取第一列的数据
y1 = data1.iloc[:, 0].apply(lambda x: round(x, 4))

# 创建横轴数据，间隔为1
x1 = range(1, len(y1) + 1)

# 从第二个CSV文件加载数据
#data2 = pd.read_csv('csv/mec_data_1114/plot_20231114_111259/exe_traj.csv')
#data2 = pd.read_csv('csv/mec_data_1114_sim/sim_plot_20231114_162048/exe_traj.csv')
# data2 = pd.read_csv('csv/mec_data_1114_sim/sim_plot_20231114_171054/exe_traj.csv')
#data2 = pd.read_csv('csv/mec_data_1114_sim/sim_plot_20231114_172304/exe_traj.csv')
#data2 = pd.read_csv('csv/mec_data_1114_sim/sim_plot_20231114_173045/exe_traj.csv')
#data2 = pd.read_csv('csv/mec_data_1114_sim/sim_plot_20231114_184555/exe_traj.csv')
#data2 = pd.read_csv('csv/mec_data_1114_sim/sim_plot_20231114_205948/exe_traj.csv')
data2 = pd.read_csv('/home/diamondlee/mec-arm-ws/mec_mpc_sim/src/mec_speed_mpc/record_csv/sim_plot_20231117_111608/exe_traj.csv')


# # 提取第三列的数据并保留小数点后三位
# data2.iloc[1:, 2] = data2.iloc[1:, 2].apply(lambda x: round(x, 4))

# 提取第三列，第二行开始后的数据
y2 = data2.iloc[:, 2].apply(lambda x: round(x, 4))  # 前面指的是第二行，2是指的是第三列

# 创建横轴数据，间隔为1
x2 = range(2, len(y2) + 2) #!delay:1 non:2

#获得prediction_traj
#data3 = pd.read_csv('csv/mec_data_1114/plot_20231114_111259/prediction_traj.csv')
#data3 = pd.read_csv('csv/mec_data_1114_sim/sim_plot_20231114_162048/prediction_traj.csv')
#data3 = pd.read_csv('csv/mec_data_1114_sim/sim_plot_20231114_171054/prediction_traj.csv')
#data3 = pd.read_csv('csv/mec_data_1114_sim/sim_plot_20231114_172304/prediction_traj.csv')
#data3 = pd.read_csv('csv/mec_data_1114_sim/sim_plot_20231114_173045/prediction_traj.csv')
#data3 = pd.read_csv('csv/mec_data_1114_sim/sim_plot_20231114_184555/prediction_traj.csv')
#data3 = pd.read_csv('csv/mec_data_1114_sim/sim_plot_20231114_205948/prediction_traj.csv')
data3 = pd.read_csv('/home/diamondlee/mec-arm-ws/mec_mpc_sim/src/mec_speed_mpc/record_csv/sim_plot_20231117_111608/prediction_traj.csv')

# 创建折线图
plt.plot(x1, y1, label='REAL', marker='o', markersize=10, markerfacecolor='red', linewidth=5)
plt.plot(x2, y2, label='EXE', marker='s', markersize=10, markerfacecolor='blue', linewidth=5)

# x3 = range(1,9)
# selected_rows = data3.iloc[0:8, 0]
# plt.plot(x3, selected_rows, label='PRED1', marker='^', markersize=10, markerfacecolor='green', linewidth=2, linestyle='--')
# for i, j in zip(x3, selected_rows):
#     plt.text(i, j, str(j), ha='right', va='bottom', fontsize=15, rotation=0)

# 添加标题和标签
plt.title('REAL-EXE')
plt.xlabel('loop_cnt')
plt.ylabel('pos_X')
# 调整坐标轴刻度间隔
plt.xticks(range(1, max(len(x1),len(x2)) + 10), rotation=45)  # 设置x轴刻度间隔为1，可根据需求调整旋转角度
y_ticks = np.arange(-1.0,
                    0.23, 0.01)
plt.yticks(y_ticks, [f'{i:.3f}' for i in y_ticks])


# # 在每个数据点上标出数值
# for i, j in zip(x1, y1):
#     plt.text(i, j, str(j), ha='right', va='bottom', fontsize=15, rotation=45)
# for i, j in zip(x2, y2):
#     plt.text(i, j, str(j), ha='right', va='bottom', fontsize=15, rotation=0)

####################################PREDICT###########################################3
x3_start = 1 #!dalay:3 non:1
mpc_horizon = 9 - 1
# 画出每一段预测的traj曲线
for i in range(1, len(data3), 30):
    selected_rows = data3.iloc[i:i + mpc_horizon, 0].apply(lambda x: round(x, 4))
    x3 = range(x3_start, len(selected_rows) + x3_start)
    x3_start += 1
    plt.plot(x3, selected_rows, label='PRED' + str(x3_start - 2), marker='^', markersize=10, markerfacecolor='orange', linewidth=2, linestyle='--')
    # for o, j in zip(x3, selected_rows):
    #     plt.text(o, j, str(j), ha='left', va='bottom', fontsize=15, rotation=0)
############################################################################################
# 添加网格
plt.grid(True)

# 显示图例
plt.legend()

# 显示图表
plt.show()


# # 调整坐标轴刻度间隔
# plt.xticks(range(1, max(len(x1),len(x2)) + 1), rotation=45)  # 设置x轴刻度间隔为1，可根据需求调整旋转角度
# #plt.yticks(y2, fontsize=8)  # 设置y轴刻度为数据点的值，同时调整刻度字体大小
# # plt.yticks(fontsize=8)
# # plt.yticks([round(i, 5) for i in plt.yticks()[0]])  # 设置y轴刻度间隔为0.0001，并保留四位小数
# # # 显式设置 y 轴刻度间隔为 0.0001，并保留四位小数
# # plt.yticks([i * 0.0001 for i in range(int(min(y1) * 10000), int(max(y1 * 10000) + 1))],
# #            [f'{i:.4f}' for i in plt.yticks()[0]])
# # # 显式设置 y 轴刻度间隔为 0.0001，并保留四位小数
# # plt.yticks([i * 0.0001 for i in range(int(min(y1) * 10000), int(max(y2) * 10000) + 1)],
# #            [f'{i:.4f}' for i in plt.yticks()[0]])
# # # 显式设置 y 轴刻度间隔为 0.0001，并保留四位小数
# # plt.locator_params(axis='y', nbins=7)

# # y_ticks = np.arange(min(min(y1), min(y2)),
# #                     max(max(y1), max(y2)), 0.01)
# y_ticks = np.arange(-0.03,
#                     0.23, 0.01)
# plt.yticks(y_ticks, [f'{i:.3f}' for i in y_ticks])


# # 在每个数据点上标出数值
# for i, j in zip(x1, y1):
#     plt.text(i, j, str(j), ha='right', va='bottom', fontsize=15, rotation=45)
# for i, j in zip(x2, y2):
#     plt.text(i, j, str(j), ha='right', va='bottom', fontsize=15, rotation=-45)

# # 添加网格
# plt.grid(True)

# # 显示图例
# plt.legend()

# # 显示图表
# plt.show()
