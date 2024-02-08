# import pandas as pd
# import numpy as np
# import matplotlib.pyplot as plt
# from scipy.interpolate import make_interp_spline

# # 读取data3.csv文件
# data3 = pd.read_csv('csv/mec_data/record_csv/plot_20231111_182947/prediction_traj.csv')

# # 按规律获取指定行的数据
# #selected_rows = []
# # for i in range(1, len(data3), 39):
# #     end_row = i + 8 #8就是预测时间区域的长度
# #     selected_rows.append(data3.iloc[i:end_row, 0])
# selected_rows = data3.iloc[1:9, 0]

# # 创建横轴数据，间隔为1
# x3 = range(1, 1 + len(selected_rows))  # 从第二行开始

# # 创建折线图
# plt.plot(x3, selected_rows, label='数据3 - 第一列', marker='^', markersize=10, markerfacecolor='orange', linewidth=2)

# # 添加标题和标签
# plt.title('三个CSV文件的第一列数据折线图')
# plt.xlabel('横轴')
# plt.ylabel('第一列数据')

# # 调整坐标轴刻度间隔
# plt.xticks(range(1, len(x3) + 1), rotation=45)

# # 添加网格
# plt.grid(True)

# # 显示图例
# plt.legend()

# # 显示图表
# plt.show()

# import pandas as pd
# import matplotlib.pyplot as plt

# # 读取data3.csv文件
# data3 = pd.read_csv('csv/mec_data/record_csv/plot_20231111_182947/prediction_traj.csv')

# # 按规律获取指定行的数据
# selected_rows = []
# for i in range(1, len(data3), 39):
#     selected_rows.extend(data3.iloc[i:i+8, 0])

# # 创建横轴数据，间隔为1
# x3 = range(1, 1 + len(selected_rows))  # 从第二行开始

# # 创建折线图
# plt.plot(x3, selected_rows, label='数据3 - 第一列', marker='^', markersize=10, markerfacecolor='orange', linewidth=2)

# # 添加标题和标签
# plt.title('数据3的特定行第一列数据折线图')
# plt.xlabel('横轴')
# plt.ylabel('第一列数据')

# # 调整坐标轴刻度间隔
# plt.xticks(range(1, len(x3) + 1), rotation=45)

# # 添加网格
# plt.grid(True)

# # 显示图例
# plt.legend()

# # 显示图表
# plt.show()

import pandas as pd
import matplotlib.pyplot as plt

# 读取data3.csv文件
data3 = pd.read_csv('csv/mec_data/record_csv/plot_20231111_182947/prediction_traj.csv')

# 按规律获取指定行的数据
selected_rows = []
selected_indices = []
for i in range(2, len(data3), 39):
    rows = data3.iloc[i:i+8, 0]
    selected_rows.extend(rows)
    selected_indices.extend(range(i, i+8))

# 创建折线图
plt.plot(selected_indices, selected_rows, label='数据3 - 第一列', marker='^', markersize=10, markerfacecolor='orange', linewidth=2)

# 添加标题和标签
plt.title('数据3的特定行第一列数据折线图')
plt.xlabel('横轴')
plt.ylabel('第一列数据')

# 调整坐标轴刻度间隔
plt.xticks(selected_indices, rotation=45)

# 添加网格
plt.grid(True)

# 显示图例
plt.legend()

# 显示图表
plt.show()