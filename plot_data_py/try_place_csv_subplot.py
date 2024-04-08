
#READ 保存速度和时间到CSV
# import pandas as pd
# import matplotlib.pyplot as plt
# import numpy as np

# def ewma(column):
#     return column.ewm(alpha=0.075).mean()

# # 读取第一个CSV文件，提取前两列
# df1 = pd.read_csv('/home/diamondlee/VKConTieta_ws/src/open_door_tieta_mpc/positon_results/mpc_position_20240321_114338/mpc_all_joints_positions.csv', header=None, index_col=False)
# #取df1的前两列
# df1 = df1.values.tolist()
# df1 = [x[:2] for x in df1]
# #print(df1)
# # 读取第二个CSV文件，提取所有列
# df2 = pd.read_csv('/home/diamondlee/VKConTieta_ws/src/open_door_tieta_mpc/positon_results/ik_position_20240321_114706/ik_theta_URjoints_positions.csv', header=None, index_col=False)
# #print(df2)
# df2 = df2.values.tolist()
# # 合并两个DataFrame
# #df = pd.concat([df1, df2], axis=1, ignore_index=True)
# df = [x+y for x, y in zip(df1, df2)]

# #保存df的第一行
# csv_pos = df[0]
# # df.insert(0, df[0])



# #添加最后一行元素
# #df.append(df[-1])

# #print(df)
# df = pd.DataFrame(df)
# #print(df)

# # 计算速度，两两作差除以0.05
# velocity = df.diff()/0.036968838526912184
# velocity_1 = df.diff()/0.05
# #print(velocity)

# velocity = velocity.dropna(axis=0)
# velocity_1 = velocity_1.dropna(axis=0)

# # 假设 df 是你的 DataFrame，速度列名为 'velocity'
# # 速度单位应该是位移/时间间隔，因此位移就是速度 * 时间间隔
# # 使用 pandas 的 cumsum() 方法计算累积位移
# # displacement = (velocity_1[3] * 0.05).cumsum()

# # # 最终的位移就是累积位移的最后一个值
# # total_displacement = displacement.iloc[-1]

# # print("Total displacement:", total_displacement)
# #print(velocity)plt.scatter(np.arange(38,38+len(velocity[3])), velocity[3], color='blue', label='Original Velocity')
# # plt.plot
# # zeros_row = pd.DataFrame([[0]*len(velocity.columns)], columns=velocity.columns)
# # velocity = pd.concat([velocity,zeros_row], ignore_index=True)
# colors = ['blue','green', 'orange', 'purple','cyan', 'pink',  'brown', 'red', 'olive']
# labels = ['X', 'Y', 'Theta','Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6']


# list_new_vels = []

# # 创建 DataFrame
# for i in range(9):
#     df = pd.DataFrame({'time': range(41), 'value': [velocity.iloc[-1,i],velocity.iloc[-1,i],velocity.iloc[-1,i], np.nan,np.nan,np.nan,np.nan, velocity.iloc[-1,i]*0.95,np.nan, np.nan, np.nan,np.nan,np.nan, np.nan, np.nan, np.nan,np.nan,np.nan,np.nan,np.nan, np.nan,np.nan,np.nan, np.nan,np.nan,velocity.iloc[-1,i]*0.20, np.nan, np.nan,np.nan,np.nan, np.nan,velocity.iloc[-1,i]*0.02,np.nan, np.nan,np.nan,np.nan,velocity.iloc[-1,i]*0.003, np.nan,np.nan,0.0,0.0]})

#     df_start = pd.DataFrame({'time': range(41), 'value': [velocity.iloc[2,i],velocity.iloc[1,i],velocity.iloc[1,i], np.nan,np.nan,np.nan,np.nan, velocity.iloc[0,i]*0.45,np.nan, np.nan, np.nan,np.nan,np.nan, np.nan, np.nan, np.nan,np.nan,np.nan,np.nan,np.nan, np.nan,np.nan,np.nan, np.nan,np.nan,velocity.iloc[0,i]*0.08, np.nan, np.nan,np.nan,np.nan, np.nan,velocity.iloc[0,i]*0.01,np.nan, np.nan,np.nan,np.nan,velocity.iloc[0,i]*0.003, np.nan,np.nan,0.0,0.0]})


#     # 创建一个包含所有时间点的新 DataFrame
#     time_range = pd.DataFrame({'time': range(41)})
#     time_range_start = pd.DataFrame({'time': range(41)})

#     # 合并两个 DataFrame，使用 time 列作为连接键
#     merged_df = pd.merge(time_range, df, on='time', how='left')
#     merged_df_start = pd.merge(time_range_start, df_start, on='time', how='left')

#     # 对 value 列进行插值
#     interpolated_df = merged_df.interpolate(method='polynomial',order=3)
#     interpolated_df_start = merged_df_start.interpolate(method='polynomial',order=3)

#     interpolated_df_start[interpolated_df_start.columns[::-1]] = interpolated_df_start[interpolated_df_start.columns[::-1]].values[::-1]
# #interpolated_df_start = interpolated_df_start.iloc[:,::-1]

# # # 输出插值后的 DataFrame
# # print("\n插值后的 DataFrame:")
# # print(interpolated_df)


# # # 输出插值后的 DataFrame
# # print(interpolated_df)
#     final_df = interpolated_df['value'].tail(38)
# # print(final_df)

#     final_plot_df = pd.concat([velocity[i],final_df],ignore_index=True)

#     init_df = interpolated_df_start['value'].iloc[8:38]

#     final_plot_df_start = pd.concat([init_df,final_plot_df],ignore_index=True)
# # print(final_plot_df)
# # print(final_plot_df.index)
#     final_plot_df.astype('float16')
# #final_plot_df = final_plot_df.to_frame()
#     smoothed_velocity = (final_plot_df.to_frame()).apply(ewma)
#     smoothed_velocity_start = (final_plot_df_start.to_frame()).apply(ewma)


#     df_long = pd.DataFrame({'time': range(23), 'value': [smoothed_velocity_start.iloc[-7,0],smoothed_velocity_start.iloc[-6,0],smoothed_velocity_start.iloc[-5,0],smoothed_velocity_start.iloc[-4,0],smoothed_velocity_start.iloc[-3,0],smoothed_velocity_start.iloc[-2,0],smoothed_velocity_start.iloc[-1,0], np.nan,np.nan,np.nan,np.nan,np.nan,np.nan,np.nan,np.nan,np.nan,np.nan,np.nan,np.nan,0.1*smoothed_velocity_start.iloc[-1,0],np.nan,0.0,0.0]})
#     time_range_long = pd.DataFrame({'time': range(23)})

#     # 合并两个 DataFrame，使用 time 列作为连接键
#     merged_df_long = pd.merge(time_range_long, df_long, on='time', how='left')

#     interpolated_df_long = merged_df_long.interpolate(method='polynomial',order=3)
#     final_plot_df_long = pd.concat([smoothed_velocity_start,interpolated_df_long['value'].tail(16)],ignore_index=True)

#     new_timestep = 0.05 * len(velocity[3]) / len(final_plot_df_long)

#     x_long = [0+new_timestep*j for j in range(len(final_plot_df_long))]
#     # plt.plot(x_long,final_plot_df_long, color=colors[i], label=labels[i],linewidth=2.5)
#     list_new_vels.append(final_plot_df_long)

# x_tune = pd.DataFrame(x_long)
# list_new_vels.append(x_tune)


# # print(interpolated_df_end)
# # print((velocity_1.iloc[:,velocity_col]).tail(20))
# # 创建一个新的 DataFrame，其中每个 DataFrame 的数据占据一列
# combined_df = pd.DataFrame()

# # 将每个 DataFrame 的数据作为新列添加到 combined_df
# for i, df in enumerate(list_new_vels):
#     combined_df[f'Data_{i}'] = df.values.flatten()

# # 保存 combined_df 到 CSV 文件
# combined_df.to_csv('place/velocity_combined_data_and_time.csv', index=False)

# import pandas as pd
# import matplotlib.pyplot as plt
# import numpy as np

# def ewma(column):
#     return column.ewm(alpha=0.075).mean()

# # 读取第一个CSV文件，提取前两列
# df1 = pd.read_csv('/home/diamondlee/VKConTieta_ws/src/open_door_tieta_mpc/positon_results/mpc_position_20240319_193553/mpc_all_joints_positions.csv', header=None, index_col=False)
# #取df1的前两列
# df1 = df1.values.tolist()
# df1 = [x[:2] for x in df1]
# #print(df1)
# # 读取第二个CSV文件，提取所有列
# df2 = pd.read_csv('/home/diamondlee/VKConTieta_ws/src/open_door_tieta_mpc/positon_results/ik_position_20240319_195550/ik_theta_URjoints_positions.csv', header=None, index_col=False)
# #print(df2)
# df2 = df2.values.tolist()
# # 合并两个DataFrame
# #df = pd.concat([df1, df2], axis=1, ignore_index=True)
# df = [x+y for x, y in zip(df1, df2)]
# # df.insert(0, df[0])



# #添加最后一行元素
# #df.append(df[-1])

# #print(df)
# df = pd.DataFrame(df)
# #print(df)

# # 计算速度，两两作差除以0.05
# velocity = df.diff()/0.0377906976744186
# velocity_1 = df.diff()/0.05
# #print(velocity)

# velocity = velocity.dropna(axis=0)
# velocity_1 = velocity_1.dropna(axis=0)

# # 假设 df 是你的 DataFrame，速度列名为 'velocity'
# # 速度单位应该是位移/时间间隔，因此位移就是速度 * 时间间隔
# # 使用 pandas 的 cumsum() 方法计算累积位移
# displacement = (velocity_1[8] * 0.05).cumsum()

# # 最终的位移就是累积位移的最后一个值
# total_displacement = displacement.iloc[-1]


# colors = ['blue','green', 'orange', 'purple','cyan', 'pink',  'brown', 'red', 'olive']
# labels = ['X', 'Y', 'Theta','Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6']
# # # print(velocity)
# # print(velocity)
# plt.figure(figsize=(8, 6.6))
# plt.xlabel('Time (s)',fontweight='bold')
# plt.ylabel('Magnitude',fontweight='bold')
# plt.title('Place Action Velocitys Over Time',fontweight='bold')
# x_ticks = [0, 2, 4, 6, 8, 10, 12, 13.0 ]
# plt.xlim(0,13.0)
# #y 间隔0.1 -0.5到0.5
# y_ticks = [-0.5, -0.4, -0.3, -0.2, -0.1, 0, 0.1, 0.2, 0.3, 0.4, 0.5]
# plt.ylim(-0.5, 0.5)


# # 设置x和y轴的刻度
# plt.xticks(x_ticks, [f'{tick:.1f}' for tick in x_ticks])
# plt.yticks(y_ticks, [f'{tick:.1f}' for tick in y_ticks])

# list_new_vels = []

# # 创建 DataFrame
# for i in range(9):
#     df = pd.DataFrame({'time': range(41), 'value': [velocity.iloc[-3,i],velocity.iloc[-2,i],velocity.iloc[-1,i], np.nan,np.nan,np.nan,np.nan, velocity.iloc[-1,i]*0.95,np.nan, np.nan, np.nan,np.nan,np.nan, np.nan, np.nan, np.nan,np.nan,np.nan,np.nan,np.nan, np.nan,np.nan,np.nan, np.nan,np.nan,velocity.iloc[-1,i]*0.20, np.nan, np.nan,np.nan,np.nan, np.nan,velocity.iloc[-1,i]*0.02,np.nan, np.nan,np.nan,np.nan,velocity.iloc[-1,i]*0.003, np.nan,np.nan,0.0,0.0]})

#     df_start = pd.DataFrame({'time': range(41), 'value': [velocity.iloc[2,i],velocity.iloc[1,i],velocity.iloc[1,i], np.nan,np.nan,np.nan,np.nan, velocity.iloc[0,i]*0.45,np.nan, np.nan, np.nan,np.nan,np.nan, np.nan, np.nan, np.nan,np.nan,np.nan,np.nan,np.nan, np.nan,np.nan,np.nan, np.nan,np.nan,velocity.iloc[0,i]*0.08, np.nan, np.nan,np.nan,np.nan, np.nan,velocity.iloc[0,i]*0.01,np.nan, np.nan,np.nan,np.nan,velocity.iloc[0,i]*0.003, np.nan,np.nan,0.0,0.0]})


#     # 创建一个包含所有时间点的新 DataFrame
#     time_range = pd.DataFrame({'time': range(41)})
#     time_range_start = pd.DataFrame({'time': range(41)})

#     # 合并两个 DataFrame，使用 time 列作为连接键
#     merged_df = pd.merge(time_range, df, on='time', how='left')
#     merged_df_start = pd.merge(time_range_start, df_start, on='time', how='left')

#     # 对 value 列进行插值
#     interpolated_df = merged_df.interpolate(method='polynomial',order=3)
#     interpolated_df_start = merged_df_start.interpolate(method='polynomial',order=3)

#     interpolated_df_start[interpolated_df_start.columns[::-1]] = interpolated_df_start[interpolated_df_start.columns[::-1]].values[::-1]
# #interpolated_df_start = interpolated_df_start.iloc[:,::-1]

# # # 输出插值后的 DataFrame
# # print("\n插值后的 DataFrame:")
# # print(interpolated_df)


# # # 输出插值后的 DataFrame
# # print(interpolated_df)
#     final_df = interpolated_df['value'].tail(38)
# # print(final_df)

#     final_plot_df = pd.concat([velocity[i],final_df],ignore_index=True)

#     #init_df = interpolated_df_start['value'].head(38)
#     init_df = interpolated_df_start['value'].iloc[8:38]

#     final_plot_df_start = pd.concat([init_df,final_plot_df],ignore_index=True)
# # print(final_plot_df)
# # print(final_plot_df.index)
#     final_plot_df.astype('float16')
# #final_plot_df = final_plot_df.to_frame()
#     smoothed_velocity = (final_plot_df.to_frame()).apply(ewma)
#     smoothed_velocity_start = (final_plot_df_start.to_frame()).apply(ewma)


#     df_long = pd.DataFrame({'time': range(23), 'value': [smoothed_velocity_start.iloc[-7,0],smoothed_velocity_start.iloc[-6,0],smoothed_velocity_start.iloc[-5,0],smoothed_velocity_start.iloc[-4,0],smoothed_velocity_start.iloc[-3,0],smoothed_velocity_start.iloc[-2,0],smoothed_velocity_start.iloc[-1,0], np.nan,np.nan,np.nan,np.nan,np.nan,np.nan,np.nan,np.nan,np.nan,np.nan,np.nan,np.nan,0.1*smoothed_velocity_start.iloc[-1,0],np.nan,0.0,0.0]})
#     time_range_long = pd.DataFrame({'time': range(23)})

#     # 合并两个 DataFrame，使用 time 列作为连接键
#     merged_df_long = pd.merge(time_range_long, df_long, on='time', how='left')

#     interpolated_df_long = merged_df_long.interpolate(method='polynomial',order=3)
#     final_plot_df_long = pd.concat([smoothed_velocity_start,interpolated_df_long['value'].tail(16)],ignore_index=True)

#     new_timestep = 0.05 * len(velocity[3]) / len(final_plot_df_long)

#     x_long = [0+new_timestep*j for j in range(len(final_plot_df_long))]
#     # plt.plot(x_long,final_plot_df_long, color=colors[i], label=labels[i],linewidth=2.5)
#     list_new_vels.append(final_plot_df_long)
# x_tune = pd.DataFrame(x_long)
# list_new_vels.append(x_tune)


# # print(interpolated_df_end)
# # print((velocity_1.iloc[:,velocity_col]).tail(20))
# # 创建一个新的 DataFrame，其中每个 DataFrame 的数据占据一列
# combined_df = pd.DataFrame()

# # 将每个 DataFrame 的数据作为新列添加到 combined_df
# for i, df in enumerate(list_new_vels):
#     combined_df[f'Data_{i}'] = df.values.flatten()

# # 保存 combined_df 到 CSV 文件
# combined_df.to_csv('place/place_velocity_combined_data_and_time.csv', index=False)

#READ 保存位置到CSV，时间用速度的
# import pandas as pd
# import matplotlib.pyplot as plt
# import numpy as np

# def ewma(column):
#     return column.ewm(alpha=0.075).mean()

# # 读取第一个CSV文件，提取前两列
# df1 = pd.read_csv('/home/diamondlee/VKConTieta_ws/src/open_door_tieta_mpc/positon_results/mpc_position_20240319_193553/mpc_all_joints_positions.csv', header=None, index_col=False)
# #取df1的前两列
# df1 = df1.values.tolist()
# df1 = [x[:2] for x in df1]
# #print(df1)
# # 读取第二个CSV文件，提取所有列
# df2 = pd.read_csv('/home/diamondlee/VKConTieta_ws/src/open_door_tieta_mpc/positon_results/ik_position_20240319_195550/ik_theta_URjoints_positions.csv', header=None, index_col=False)
# #print(df2)
# df2 = df2.values.tolist()
# # 合并两个DataFrame
# #df = pd.concat([df1, df2], axis=1, ignore_index=True)
# df = [x+y for x, y in zip(df1, df2)]
# # df.insert(0, df[0])

# csv_pos = df[0]

# #添加最后一行元素
# #df.append(df[-1])

# #print(df)
# df = pd.DataFrame(df)
# #print(df)

# # 计算速度，两两作差除以0.05
# velocity = df.diff()/0.0377906976744186
# velocity_1 = df.diff()/0.05
# #print(velocity)

# velocity = velocity.dropna(axis=0)
# velocity_1 = velocity_1.dropna(axis=0)
# #print(velocity)


# # 假设 df 是你的 DataFrame，速度列名为 'velocity'
# # 速度单位应该是位移/时间间隔，因此位移就是速度 * 时间间隔
# # 使用 pandas 的 cumsum() 方法计算累积位移
# # displacement = (velocity_1[3] * 0.05).cumsum()

# # # 最终的位移就是累积位移的最后一个值
# # total_displacement = displacement.iloc[-1]

# # print("Total displacement:", total_displacement)
# #print(velocity)plt.scatter(np.arange(38,38+len(velocity[3])), velocity[3], color='blue', label='Original Velocity')
# # plt.plot
# # zeros_row = pd.DataFrame([[0]*len(velocity.columns)], columns=velocity.columns)
# # velocity = pd.concat([velocity,zeros_row], ignore_index=True)
# colors = ['blue','green', 'orange', 'purple','cyan', 'pink',  'brown', 'red', 'olive']
# labels = ['X', 'Y', 'Theta','Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6']


# list_new_vels = []

# # 创建 DataFrame
# for i in range(9):
#     df = pd.DataFrame({'time': range(41), 'value': [velocity.iloc[-1,i],velocity.iloc[-1,i],velocity.iloc[-1,i], np.nan,np.nan,np.nan,np.nan, velocity.iloc[-1,i]*0.95,np.nan, np.nan, np.nan,np.nan,np.nan, np.nan, np.nan, np.nan,np.nan,np.nan,np.nan,np.nan, np.nan,np.nan,np.nan, np.nan,np.nan,velocity.iloc[-1,i]*0.20, np.nan, np.nan,np.nan,np.nan, np.nan,velocity.iloc[-1,i]*0.02,np.nan, np.nan,np.nan,np.nan,velocity.iloc[-1,i]*0.003, np.nan,np.nan,0.0,0.0]})

#     df_start = pd.DataFrame({'time': range(41), 'value': [velocity.iloc[2,i],velocity.iloc[1,i],velocity.iloc[1,i], np.nan,np.nan,np.nan,np.nan, velocity.iloc[0,i]*0.45,np.nan, np.nan, np.nan,np.nan,np.nan, np.nan, np.nan, np.nan,np.nan,np.nan,np.nan,np.nan, np.nan,np.nan,np.nan, np.nan,np.nan,velocity.iloc[0,i]*0.08, np.nan, np.nan,np.nan,np.nan, np.nan,velocity.iloc[0,i]*0.01,np.nan, np.nan,np.nan,np.nan,velocity.iloc[0,i]*0.003, np.nan,np.nan,0.0,0.0]})


#     # 创建一个包含所有时间点的新 DataFrame
#     time_range = pd.DataFrame({'time': range(41)})
#     time_range_start = pd.DataFrame({'time': range(41)})

#     # 合并两个 DataFrame，使用 time 列作为连接键
#     merged_df = pd.merge(time_range, df, on='time', how='left')
#     merged_df_start = pd.merge(time_range_start, df_start, on='time', how='left')

#     # 对 value 列进行插值
#     interpolated_df = merged_df.interpolate(method='polynomial',order=3)
#     interpolated_df_start = merged_df_start.interpolate(method='polynomial',order=3)

#     interpolated_df_start[interpolated_df_start.columns[::-1]] = interpolated_df_start[interpolated_df_start.columns[::-1]].values[::-1]
# #interpolated_df_start = interpolated_df_start.iloc[:,::-1]

# # # 输出插值后的 DataFrame
# # print("\n插值后的 DataFrame:")
# # print(interpolated_df)


# # # 输出插值后的 DataFrame
# # print(interpolated_df)
#     final_df = interpolated_df['value'].tail(38)
# # print(final_df)

#     final_plot_df = pd.concat([velocity[i],final_df],ignore_index=True)

#     init_df = interpolated_df_start['value'].iloc[8:38]

#     final_plot_df_start = pd.concat([init_df,final_plot_df],ignore_index=True)
# # print(final_plot_df)
# # print(final_plot_df.index)
#     final_plot_df.astype('float16')
# #final_plot_df = final_plot_df.to_frame()
#     smoothed_velocity = (final_plot_df.to_frame()).apply(ewma)
#     smoothed_velocity_start = (final_plot_df_start.to_frame()).apply(ewma)


#     df_long = pd.DataFrame({'time': range(23), 'value': [smoothed_velocity_start.iloc[-7,0],smoothed_velocity_start.iloc[-6,0],smoothed_velocity_start.iloc[-5,0],smoothed_velocity_start.iloc[-4,0],smoothed_velocity_start.iloc[-3,0],smoothed_velocity_start.iloc[-2,0],smoothed_velocity_start.iloc[-1,0], np.nan,np.nan,np.nan,np.nan,np.nan,np.nan,np.nan,np.nan,np.nan,np.nan,np.nan,np.nan,0.1*smoothed_velocity_start.iloc[-1,0],np.nan,0.0,0.0]})
#     time_range_long = pd.DataFrame({'time': range(23)})

#     # 合并两个 DataFrame，使用 time 列作为连接键
#     merged_df_long = pd.merge(time_range_long, df_long, on='time', how='left')

#     interpolated_df_long = merged_df_long.interpolate(method='polynomial',order=3)
#     final_plot_df_long = pd.concat([smoothed_velocity_start,interpolated_df_long['value'].tail(16)],ignore_index=True)

#     new_timestep = 0.05 * len(velocity[3]) / len(final_plot_df_long)

#     x_long = [0+new_timestep*j for j in range(len(final_plot_df_long))]
#     # plt.plot(x_long,final_plot_df_long, color=colors[i], label=labels[i],linewidth=2.5)
#     list_new_vels.append(final_plot_df_long)

# list_new_poss = []

# for i in range(len(list_new_vels)):
#     # 根据速度数据计算位移
#     df_displace = list_new_vels[i] * new_timestep

#     # 计算位移的累积和，得到每个时刻的位置
#     df_position = df_displace.cumsum()

#     # 在 t=0.2s 时的位置为 10 米
#     position_at_39 = csv_pos[i]

#     # 根据 t=0.2s 时的位置来调整初始位置
#     initial_position = position_at_39 - df_position[0].iloc[30]

#     # 将初始位置加到每个时刻的位置上
#     df_position[0] += initial_position

#     list_new_poss.append(pd.DataFrame(df_position[0]))

#     # plt.plot(x_tune,df_position[0], color=colors[i], label=labels[i],linewidth=2.5)

# x_tune = pd.DataFrame(x_long)
# list_new_vels.append(x_tune)


# # print(interpolated_df_end)
# # print((velocity_1.iloc[:,velocity_col]).tail(20))
# # 创建一个新的 DataFrame，其中每个 DataFrame 的数据占据一列
# combined_df = pd.DataFrame()

# # 将每个 DataFrame 的数据作为新列添加到 combined_df
# for i, df in enumerate(list_new_poss):
#     combined_df[f'Data_{i}'] = df.values.flatten()

# # 保存 combined_df 到 CSV 文件
# combined_df.to_csv('place/place_positon_combined_data.csv', index=False)

#READ 这段是按照两个子图画出
import pandas as pd
import matplotlib.pyplot as plt

# 读取速度和位移的 CSV 文件
df_velocity = pd.read_csv('place/place_velocity_combined_data_and_time.csv')
df_displacement = pd.read_csv('place/place_positon_combined_data.csv')

# 创建一个新的 Figure 和 Axes 对象
fig, axes = plt.subplots(2, 1, figsize=(8, 10.6), gridspec_kw={'height_ratios': [2, 2]})

colors = ['blue','green', 'orange', 'purple','cyan', 'pink',  'brown', 'red', 'olive']
labels = ['X', 'Y', 'Theta','Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6']

# 绘制速度曲线子图
for i in range(9):
    axes[0].plot(df_velocity.iloc[:,9], df_velocity.iloc[:,i], label=labels[i],color=colors[i],linewidth=2.5)

x_ticks = [0, 2, 4, 6, 8, 10, 12, 13.0 ]
axes[0].set_xlim(0,13.0)
#y 间隔0.1 -0.5到0.5
y_ticks = [-0.5, -0.4, -0.3, -0.2, -0.1, 0, 0.1, 0.2, 0.3, 0.4, 0.5]
axes[0].set_ylim(-0.5, 0.5)

# 设置x和y轴的刻度
axes[0].set_xticks(x_ticks, [f'{tick:.1f}' for tick in x_ticks])
axes[0].set_yticks(y_ticks, [f'{tick:.1f}' for tick in y_ticks])

axes[0].set_title('Place Action Velocity Over Time',fontweight='bold')
axes[0].set_xlabel('Time (s)',fontweight='bold')
axes[0].set_ylabel('Magnitude',fontweight='bold')
#legend小一些  xx-small, x-small, small, medium, large, x-large, xx-large, larger, smaller, None
axes[0].legend(loc="upper left", fontsize='small')
axes[0].grid(True)

# 绘制位移曲线子图
for i in range(9):
    axes[1].plot(df_velocity.iloc[:,9], df_displacement.iloc[:,i], color=colors[i],linewidth=2.5)

axes[1].set_ylim(-2.0,3.0)

#x_ticks_pick = [0, 2, 4, 6, 8, 10, 12, 13.0 ]
axes[1].set_xlim(0.0,13.0)

#plt.text(22.8,-0.3,'22.8',fontsize=10.5,color='black',ha='left',va='top')
# 设置x和y轴的刻度
axes[1].set_xticks(x_ticks, [f'{tick:.1f}' for tick in x_ticks])

axes[1].set_title('Place Action Position Over Time',fontweight='bold')
axes[1].set_xlabel('Time (s)',fontweight='bold')
axes[1].set_ylabel('Magnitude',fontweight='bold')
#axes[1].legend()
axes[1].grid(True)

# 调整布局
plt.tight_layout()

# 显示图形
plt.show()






