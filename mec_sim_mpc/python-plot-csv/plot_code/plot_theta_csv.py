import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import make_interp_spline

# 从CSV文件加载数据
data1 = pd.read_csv('/home/diamondlee/mec-arm-ws/mec_mpc_sim/src/anglesPub/csv/retimed_traj.csv')
new_data1 = data1 * 180 / 3.1415926

# 提取第一列的数据
y1 = new_data1.iloc[:, 3].apply(lambda x: round(x, 2))

# 创建横轴数据，间隔为1
x1 = range(1, len(y1) + 1)

# 从第二个CSV文件加载数据
#data2 = pd.read_csv('csv/mec_data/record_csv/plot_20231111_182947/exe_traj.csv')
#data2 = pd.read_csv('csv/mec_data_1114/plot_20231114_111259/exe_traj.csv')
#data2 = pd.read_csv('csv/mec_data_1114_sim/sim_plot_20231114_162048/exe_traj.csv')
#data2 = pd.read_csv('csv/mec_data_1114_sim/sim_plot_20231114_171054/exe_traj.csv')
#data2 = pd.read_csv('csv/mec_data_1114_sim/sim_plot_20231114_172304/exe_traj.csv')
#data2 = pd.read_csv('csv/mec_data_1114_sim/sim_plot_20231114_173045/exe_traj.csv')
#data2 = pd.read_csv('csv/mec_data_1114_sim/sim_plot_20231114_184555/exe_traj.csv')
#data2 = pd.read_csv('csv/mec_data_1114_sim/sim_plot_20231114_191139/exe_traj.csv')
#data2 = pd.read_csv('csv/mec_data_1114_sim/sim_plot_20231114_205549/exe_traj.csv')
data2 = pd.read_csv('/home/diamondlee/mec-arm-ws/mec_mpc_sim/src/mec_speed_mpc/record_csv/sim_plot_20231117_111608/exe_traj.csv')


new_data2 = data2 * 180 / 3.1415926
# # 提取第三列的数据并保留小数点后两位

# 提取第三列，第二行开始后的数据
y2 = new_data2.iloc[:, 4].apply(lambda x: round(x, 2))  # 前面指的是第二行，2是指的是第三列

# 创建横轴数据，间隔为1
# x2 = range(1, len(y2) + 1)
x2 = range(1, len(y2) + 1)

#获得prediction_traj
#data3 = pd.read_csv('csv/mec_data/record_csv/plot_20231111_182947/prediction_traj.csv')
#data3 = pd.read_csv('csv/mec_data_1114/plot_20231114_111259/prediction_traj.csv')
#data3 = pd.read_csv('csv/mec_data_1114_sim/sim_plot_20231114_162048/prediction_traj.csv')
#data3 = pd.read_csv('csv/mec_data_1114_sim/sim_plot_20231114_171054/prediction_traj.csv')
#data3 = pd.read_csv('csv/mec_data_1114_sim/sim_plot_20231114_172304/prediction_traj.csv')
#data3 = pd.read_csv('csv/mec_data_1114_sim/sim_plot_20231114_173045/prediction_traj.csv')
#data3 = pd.read_csv('csv/mec_data_1114_sim/sim_plot_20231114_184555/prediction_traj.csv')
#data3 = pd.read_csv('csv/mec_data_1114_sim/sim_plot_20231114_191139/prediction_traj.csv')
#data3 = pd.read_csv('csv/mec_data_1114_sim/sim_plot_20231114_205549/prediction_traj.csv')
data3 = pd.read_csv('/home/diamondlee/mec-arm-ws/mec_mpc_sim/src/mec_speed_mpc/record_csv/sim_plot_20231117_111608/prediction_traj.csv')

new_data3 = data3 * 180 / 3.1415926

# 创建折线图
plt.plot(x1, y1, label='REAL', marker='o', markersize=10, markerfacecolor='red', linewidth=5)
#plt.plot(x2, y2, label='EXE', marker='s', markersize=10, markerfacecolor='blue', linewidth=5)

# x3 = range(1,9)
# selected_rows = data3.iloc[0:8, 0]
# plt.plot(x3, selected_rows, label='PRED1', marker='^', markersize=10, markerfacecolor='green', linewidth=2, linestyle='--')
# for i, j in zip(x3, selected_rows):
#     plt.text(i, j, str(j), ha='right', va='bottom', fontsize=15, rotation=0)

# 添加标题和标签
plt.title('REAL-EXE')
plt.xlabel('loop_cnt')
plt.ylabel('pos_THETA')
# 调整坐标轴刻度间隔
plt.xticks(range(1, max(len(x1),len(x2)) + 10), rotation=45)  # 设置x轴刻度间隔为1，可根据需求调整旋转角度
y_ticks = np.arange(-60.0,
                    30.0, 0.5)
plt.yticks(y_ticks, [f'{i:.3f}' for i in y_ticks])
#plt.yticks(y2, fontsize=8) #不对，y轴进行标准分化时

# # 在每个数据点上标出数值
# for i, j in zip(x1, y1):
#     plt.text(i, j, str(j), ha='right', va='bottom', fontsize=15, rotation=45)
# for i, j in zip(x2, y2):
#     plt.text(i, j, str(j), ha='right', va='bottom', fontsize=15, rotation=0)

#####################################PREDICT###########################################3
x3_start = 1
mpc_horizon = 9 - 1
# 画出每一段预测的traj曲线
# for i in range(1, len(data3), 30):
#     selected_rows = new_data3.iloc[i:i + mpc_horizon, 2].apply(lambda x: round(x, 2))
#     x3 = range(x3_start, len(selected_rows) + x3_start)
#     x3_start += 1
#     plt.plot(x3, selected_rows, label='PRED' + str(x3_start - 2), marker='^', markersize=10, markerfacecolor='orange', linewidth=2, linestyle='--')
#     # for o, j in zip(x3, selected_rows):
#     #     plt.text(o, j, str(j), ha='left', va='bottom', fontsize=15, rotation=0)
#############################################################################################
# 添加网格
plt.grid(True)

# 显示图例
plt.legend()

# 显示图表
plt.show()

