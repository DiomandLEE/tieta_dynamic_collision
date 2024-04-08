#READ 这段是将速度和时间存在csv中，时间在最后一列
# import pandas as pd
# import matplotlib.pyplot as plt
# import numpy as np

# def ewma(column):
#     return column.ewm(alpha=0.0475).mean()

# df = pd.read_csv('/home/diamondlee/VKConTieta_ws/src/tieta_mpc_sim_demo/position_results/mpc_position_20240319_115105/mpc_all_joints_positions.csv')

# df = df.iloc[:, 0:9]
# #保存df的第一行
# # csv_pos = df[0]
# csv_pos = df.iloc[0,:]
# csv_pos = csv_pos.values.tolist()
# print("csv_pos: ",csv_pos)

# # # 绘制平滑速度曲线
# # plt.figure(figsize=(12, 5))
# # plt.xlabel('Time (s)',fontweight='bold')
# # plt.ylabel('Magnitude',fontweight='bold')
# # plt.title('Pick Action Velocitys Over Time',fontweight='bold')
# # x_ticks = [0.0, 2.5, 5.0, 7.5 , 10.0, 12.5, 15, 17.5, 20.0, 22.5]
# # #间隔为3.0
# # x_ticks = [0.0, 3.0, 6.0, 9.0, 12.0, 15.0, 18.0, 21.0, 22.8]
# # plt.xlim(0.0,22.8)
# # #y 间隔0.1 -0.5到0.5
# # y_ticks = [-0.3, -0.2, -0.1, 0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
# # plt.ylim(-0.3, 0.7)
# # #plt.text(22.8,-0.3,'22.8',fontsize=10.5,color='black',ha='left',va='top')
# # # 设置x和y轴的刻度
# # plt.xticks(x_ticks, [f'{tick:.1f}' for tick in x_ticks])
# # plt.yticks(y_ticks, [f'{tick:.1f}' for tick in y_ticks])

# colors = ['blue','green', 'orange', 'purple','cyan', 'pink',  'brown', 'red', 'olive']
# labels = ['X', 'Y', 'Theta','Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6']

# velocity = df.diff()/0.024
# #print(velocity)

# velocity = velocity.dropna(axis=0)
# #print(velocity)
# # #! 现在你可以根据需要对 DataFrame 进行插值或补全，以使速度数据顺滑地趋向于零
# velocity_1 = velocity
# point_end = -23
# #velocity_col = 1
# list_new_vels = []

# for velocity_col in range(9):
#     print("point_end: ",point_end)
#     df_end = pd.DataFrame({'time': range(56), 'value': [velocity_1.iloc[point_end -2,velocity_col],np.nan,velocity_1.iloc[point_end,velocity_col],np.nan,np.nan,velocity_1.iloc[point_end,velocity_col]*0.9,np.nan,np.nan,np.nan,np.nan, np.nan, np.nan,velocity_1.iloc[point_end,velocity_col]*0.75, np.nan, np.nan,np.nan,velocity_1.iloc[point_end,velocity_col]*0.5,np.nan, np.nan,velocity_1.iloc[point_end,velocity_col]*0.35, np.nan,np.nan, np.nan,np.nan,np.nan,velocity_1.iloc[point_end,velocity_col]*0.03,np.nan,np.nan,np.nan,np.nan,np.nan,np.nan,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]})

#     df_start = pd.DataFrame({'time': range(28), 'value': [velocity_1.iloc[4,velocity_col],velocity_1.iloc[3,velocity_col],velocity_1.iloc[2,velocity_col],velocity_1.iloc[1,velocity_col],velocity_1.iloc[0,velocity_col],np.nan,velocity_1.iloc[0,velocity_col]*0.8,np.nan,np.nan,np.nan,np.nan, np.nan, np.nan,np.nan,np.nan,np.nan,velocity_1.iloc[0,velocity_col]*0.20, np.nan, np.nan,np.nan,np.nan,np.nan,velocity_1.iloc[0,velocity_col]*0.03,np.nan, np.nan,velocity_1.iloc[0,velocity_col]*0.003, np.nan,0.0]})
#     #print(velocity_col)
#     # 创建一个包含所有时间点的新 DataFrame
#     time_range_end = pd.DataFrame({'time': range(56)})
#     time_range_start = pd.DataFrame({'time': range(28)})

#     # 合并两个 DataFrame，使用 time 列作为连接键
#     merged_df_end = pd.merge(time_range_end, df_end, on='time', how='left')
#     merged_df_start = pd.merge(time_range_start, df_start, on='time', how='left')

#     # 对 value 列进行插值
#     interpolated_df_end = merged_df_end.interpolate(method='polynomial',order=3)
#     interpolated_df_start = merged_df_start.interpolate(method='polynomial',order=3)

#     interpolated_df_start[interpolated_df_start.columns[::-1]] = interpolated_df_start[interpolated_df_start.columns[::-1]].values[::-1]
#     # print(3)
#     #i
#     final_df_end = interpolated_df_end['value']
#     # print(final_df)
#     velocity = velocity_1.iloc[:(point_end -2),:]
#     print("middle Veloity: ", len(velocity.iloc[:,velocity_col]))
#     final_plot_df_end = pd.concat([velocity.iloc[:,velocity_col],final_df_end],ignore_index=True)
#     # print(4)
#     # print(final_plot_df_end)

#     init_df_start = interpolated_df_start['value'].iloc[0:23]

#     final_plot_df = pd.concat([init_df_start,final_plot_df_end],ignore_index=True)

#     smoothed_velocity = (final_plot_df.to_frame()).apply(ewma)

#     new_timestep = (0.025 * len(velocity_1) + 0.025) / len(smoothed_velocity)
#     # print("velocity_1: ", len(velocity_1))
#     # print("smoothed_velocity: ", len(smoothed_velocity))
#     # print("new time step: ", new_timestep)

#     x_tune = [0.0 + new_timestep * j for j in range(len(smoothed_velocity))]
#     # x_ = [0 + 0.025*j for j in range(len(smoothed_velocity))]
#     # i=0
#     # print(smoothed_velocity)
#     # plt.plot(x_tune, smoothed_velocity,color=colors[velocity_col], label=labels[velocity_col],linewidth=2.5)
#     list_new_vels.append(smoothed_velocity)
# x_tune = pd.DataFrame(x_tune)
# list_new_vels.append(x_tune)


# # print(interpolated_df_end)
# # print((velocity_1.iloc[:,velocity_col]).tail(20))
# # 创建一个新的 DataFrame，其中每个 DataFrame 的数据占据一列
# combined_df = pd.DataFrame()

# # 将每个 DataFrame 的数据作为新列添加到 combined_df
# for i, df in enumerate(list_new_vels):
#     combined_df[f'Data_{i}'] = df.values.flatten()

# # 保存 combined_df 到 CSV 文件
# combined_df.to_csv('combined_data_and_time.csv', index=False)

# #print(velocity_1.iloc[:,1])
# # 添加标题和图例
# plt.grid(True)
# plt.legend()

# # 显示图形
# plt.show()

#READ 这段是将位置和时间存在csv中，时间在最后一列
# import pandas as pd
# import matplotlib.pyplot as plt
# import numpy as np

# def ewma(column):
#     return column.ewm(alpha=0.0475).mean()

# df = pd.read_csv('/home/diamondlee/VKConTieta_ws/src/tieta_mpc_sim_demo/position_results/mpc_position_20240319_115105/mpc_all_joints_positions.csv')

# df = df.iloc[:, 0:9]
# #保存df的第一行
# # csv_pos = df[0]
# csv_pos = df.iloc[0,:]
# csv_pos = csv_pos.values.tolist()
# print("csv_pos: ",csv_pos)

# # # 绘制平滑速度曲线
# # plt.figure(figsize=(12, 5))
# # plt.xlabel('Time (s)',fontweight='bold')
# # plt.ylabel('Magnitude',fontweight='bold')
# # plt.title('Pick Action Velocitys Over Time',fontweight='bold')
# # x_ticks = [0.0, 2.5, 5.0, 7.5 , 10.0, 12.5, 15, 17.5, 20.0, 22.5]
# # #间隔为3.0
# # x_ticks = [0.0, 3.0, 6.0, 9.0, 12.0, 15.0, 18.0, 21.0, 22.8]
# # plt.xlim(0.0,22.8)
# # #y 间隔0.1 -0.5到0.5
# # y_ticks = [-0.3, -0.2, -0.1, 0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
# # plt.ylim(-0.3, 0.7)
# # #plt.text(22.8,-0.3,'22.8',fontsize=10.5,color='black',ha='left',va='top')
# # # 设置x和y轴的刻度
# # plt.xticks(x_ticks, [f'{tick:.1f}' for tick in x_ticks])
# # plt.yticks(y_ticks, [f'{tick:.1f}' for tick in y_ticks])

# colors = ['blue','green', 'orange', 'purple','cyan', 'pink',  'brown', 'red', 'olive']
# labels = ['X', 'Y', 'Theta','Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6']

# velocity = df.diff()/0.024
# #print(velocity)

# velocity = velocity.dropna(axis=0)
# #print(velocity)
# # #! 现在你可以根据需要对 DataFrame 进行插值或补全，以使速度数据顺滑地趋向于零
# velocity_1 = velocity
# point_end = -23
# #velocity_col = 1
# list_new_vels = []

# for velocity_col in range(9):
#     print("point_end: ",point_end)
#     df_end = pd.DataFrame({'time': range(56), 'value': [velocity_1.iloc[point_end -2,velocity_col],np.nan,velocity_1.iloc[point_end,velocity_col],np.nan,np.nan,velocity_1.iloc[point_end,velocity_col]*0.9,np.nan,np.nan,np.nan,np.nan, np.nan, np.nan,velocity_1.iloc[point_end,velocity_col]*0.75, np.nan, np.nan,np.nan,velocity_1.iloc[point_end,velocity_col]*0.5,np.nan, np.nan,velocity_1.iloc[point_end,velocity_col]*0.35, np.nan,np.nan, np.nan,np.nan,np.nan,velocity_1.iloc[point_end,velocity_col]*0.03,np.nan,np.nan,np.nan,np.nan,np.nan,np.nan,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]})

#     df_start = pd.DataFrame({'time': range(28), 'value': [velocity_1.iloc[4,velocity_col],velocity_1.iloc[3,velocity_col],velocity_1.iloc[2,velocity_col],velocity_1.iloc[1,velocity_col],velocity_1.iloc[0,velocity_col],np.nan,velocity_1.iloc[0,velocity_col]*0.8,np.nan,np.nan,np.nan,np.nan, np.nan, np.nan,np.nan,np.nan,np.nan,velocity_1.iloc[0,velocity_col]*0.20, np.nan, np.nan,np.nan,np.nan,np.nan,velocity_1.iloc[0,velocity_col]*0.03,np.nan, np.nan,velocity_1.iloc[0,velocity_col]*0.003, np.nan,0.0]})
#     #print(velocity_col)
#     # 创建一个包含所有时间点的新 DataFrame
#     time_range_end = pd.DataFrame({'time': range(56)})
#     time_range_start = pd.DataFrame({'time': range(28)})

#     # 合并两个 DataFrame，使用 time 列作为连接键
#     merged_df_end = pd.merge(time_range_end, df_end, on='time', how='left')
#     merged_df_start = pd.merge(time_range_start, df_start, on='time', how='left')

#     # 对 value 列进行插值
#     interpolated_df_end = merged_df_end.interpolate(method='polynomial',order=3)
#     interpolated_df_start = merged_df_start.interpolate(method='polynomial',order=3)

#     interpolated_df_start[interpolated_df_start.columns[::-1]] = interpolated_df_start[interpolated_df_start.columns[::-1]].values[::-1]
#     # print(3)
#     #i
#     final_df_end = interpolated_df_end['value']
#     # print(final_df)
#     velocity = velocity_1.iloc[:(point_end -2),:]
#     print("middle Veloity: ", len(velocity.iloc[:,velocity_col]))
#     final_plot_df_end = pd.concat([velocity.iloc[:,velocity_col],final_df_end],ignore_index=True)
#     # print(4)
#     # print(final_plot_df_end)

#     init_df_start = interpolated_df_start['value'].iloc[0:23]

#     final_plot_df = pd.concat([init_df_start,final_plot_df_end],ignore_index=True)

#     smoothed_velocity = (final_plot_df.to_frame()).apply(ewma)

#     new_timestep = (0.025 * len(velocity_1) + 0.025) / len(smoothed_velocity)
#     # print("velocity_1: ", len(velocity_1))
#     # print("smoothed_velocity: ", len(smoothed_velocity))
#     # print("new time step: ", new_timestep)

#     x_tune = [0.0 + new_timestep * j for j in range(len(smoothed_velocity))]
#     # x_ = [0 + 0.025*j for j in range(len(smoothed_velocity))]
#     # i=0
#     # print(smoothed_velocity)
#     # plt.plot(x_tune, smoothed_velocity,color=colors[velocity_col], label=labels[velocity_col],linewidth=2.5)
#     list_new_vels.append(smoothed_velocity)

# list_new_poss = []

# for i in range(len(list_new_vels)):
#     # 根据速度数据计算位移
#     df_displace = list_new_vels[i] * new_timestep

#     # 计算位移的累积和，得到每个时刻的位置
#     df_position = df_displace.cumsum()

#     # 在 t=0.2s 时的位置为 10 米
#     position_at_24 = csv_pos[i]

#     # 根据 t=0.2s 时的位置来调整初始位置
#     initial_position = position_at_24 - df_position[0].iloc[24]

#     # 将初始位置加到每个时刻的位置上
#     df_position[0] += initial_position

#     list_new_poss.append(pd.DataFrame(df_position[0]))

#     # plt.plot(x_tune,df_position[0], color=colors[i], label=labels[i],linewidth=2.5)

# x_tune = pd.DataFrame(x_tune)
# list_new_vels.append(x_tune)


# # print(interpolated_df_end)
# # print((velocity_1.iloc[:,velocity_col]).tail(20))
# # 创建一个新的 DataFrame，其中每个 DataFrame 的数据占据一列
# combined_df = pd.DataFrame()

# # 将每个 DataFrame 的数据作为新列添加到 combined_df
# for i, df in enumerate(list_new_poss):
#     combined_df[f'Data_{i}'] = df.values.flatten()

# # 保存 combined_df 到 CSV 文件
# combined_df.to_csv('positon_combined_data_and_time.csv', index=False)

# #print(velocity_1.iloc[:,1])
# # 添加标题和图例
# plt.grid(True)
# plt.legend()

# # 显示图形
# plt.show()

#READ 这段是按照两个子图画出
import pandas as pd
import matplotlib.pyplot as plt

# 读取速度和位移的 CSV 文件
df_velocity = pd.read_csv('pick/velocity_combined_data_and_time.csv')
df_displacement = pd.read_csv('pick/positon_combined_data_and_time.csv')

# 创建一个新的 Figure 和 Axes 对象
fig, axes = plt.subplots(2, 1, figsize=(8, 10.6), gridspec_kw={'height_ratios': [2, 2]})

colors = ['blue','green', 'orange', 'purple','cyan', 'pink',  'brown', 'red', 'olive']
labels = ['X', 'Y', 'Theta','Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6']

# 绘制速度曲线子图
for i in range(9):
    axes[0].plot(df_velocity.iloc[:,9], df_velocity.iloc[:,i], label=labels[i],color=colors[i],linewidth=2.5)

x_ticks = [0.0, 3.0, 6.0, 9.0, 12.0, 15.0, 18.0, 21.0, 22.8]
axes[0].set_xlim(0.0,22.8)
#y 间隔0.1 -0.5到0.5
y_ticks = [-0.3, -0.2, -0.1, 0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
axes[0].set_ylim(-0.3, 0.7)
#axes[0].set_text(22.8,-0.3,'22.8',fontsize=10.5,color='black',ha='left',va='top')
# 设置x和y轴的刻度
axes[0].set_xticks(x_ticks, [f'{tick:.1f}' for tick in x_ticks])
axes[0].set_yticks(y_ticks, [f'{tick:.1f}' for tick in y_ticks])

axes[0].set_title('Pick Action Velocity Over Time',fontweight='bold')
axes[0].set_xlabel('Time (s)',fontweight='bold')
axes[0].set_ylabel('Magnitude',fontweight='bold')
axes[0].legend(loc = "upper left" , fontsize = 'small')
axes[0].grid(True)

# 绘制位移曲线子图
for i in range(9):
    axes[1].plot(df_velocity.iloc[:,9], df_displacement.iloc[:,i], color=colors[i],linewidth=2.5)

axes[1].set_ylim(-3,3)

#x_ticks = [0.0, 3.0, 6.0, 9.0, 12.0, 15.0, 18.0, 21.0, 22.8]
axes[1].set_xlim(0.0,22.8)

#axes[1].set_text(22.8,-0.3,'22.8',fontsize=10.5,color='black',ha='left',va='top')
# 设置x和y轴的刻度
axes[1].set_xticks(x_ticks, [f'{tick:.1f}' for tick in x_ticks])
#plt.yticks(y_ticks, [f'{tick:.1f}' for tick in y_ticks])

axes[1].set_title('Pick Action Position Over Time',fontweight='bold')
axes[1].set_xlabel('Time (s)',fontweight='bold')
axes[1].set_ylabel('Magnitude',fontweight='bold')
#axes[1].legend()
axes[1].grid(True)

# 调整布局
plt.tight_layout()

# 显示图形
plt.show()