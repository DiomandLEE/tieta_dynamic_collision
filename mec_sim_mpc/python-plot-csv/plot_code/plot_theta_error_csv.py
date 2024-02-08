import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import make_interp_spline

# 从CSV文件加载数据
data1 = pd.read_csv('csv/retimed_traj_pick.csv')

# 提取第三列的数据
y1 = data1.iloc[:, 2].apply(lambda x: round(x, 4))
y1 =y1.tolist()[1:22]
# 打印 error_data
print("Error Data y1:", y1)

# 创建横轴数据，间隔为1
#x1 = range(1, len(y1) + 1)

# 从第二个CSV文件加载数据
#data2 = pd.read_csv('csv/mec_data/record_csv/plot_20231111_182947/exe_traj.csv')
data2 = pd.read_csv('csv/mec_data_1114/plot_20231114_111259/exe_traj.csv')

# # 提取第三列的数据并保留小数点后三位
# data2.iloc[1:, 2] = data2.iloc[1:, 2].apply(lambda x: round(x, 4))

# 提取第四列，第二行开始后的数据
y2 = data2.iloc[:, 4].apply(lambda x: round(x, 4))
# 打印 error_data
print("Error Data y2:", y2)

# 创建横轴数据，间隔为1
#x2 = range(1, len(y2) + 1)

new_y1 = y1[:20]
new_y2 = y2[:20]

error_data = (new_y2 - new_y1)#实现单位为deg
error_data.apply(lambda x: round(x, 2))
# 打印 error_data
print("Error Data:", error_data)

x = range(1, len(error_data) + 1)

# 创建折线图
plt.plot(x, error_data, label='ERROR', marker='o', markersize=10, markerfacecolor='red', linewidth=5)

# 添加标题和标签
plt.title('ERROR-Curve')
plt.xlabel('loop_cnt')
plt.ylabel('ERROR_X')

# # 在 y 轴上方显示具体的数值
# for i, txt in enumerate(error_data):
#     plt.text(0, error_data[i], f'{txt:.4f}', ha='right', va='center')
# 设置 y 轴刻度的位置和标签
plt.yticks(error_data, [f'{i:.4f}' for i in error_data])

# 调整坐标轴刻度间隔
plt.xticks(range(1, len(x) + 1), rotation=45)  # 设置x轴刻度间隔为1，可根据需求调整旋转角度
# y_ticks = np.arange(-0.03,
#                     0.23, 0.01)
# plt.yticks(y_ticks, [f'{i:.3f}' for i in y_ticks])
#plt.yticks(error_data, fontsize=12) #不对y轴进行标准分化时

# 添加网格
plt.grid(True)

# 显示图例
plt.legend()

# 显示图表
plt.show()