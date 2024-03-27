import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def ewma(column):
    return column.ewm(alpha=0.0475).mean()

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
# df.insert(0, df[0])
# # df.insert(0, df[0])
# # df.insert(0, df[0])
# # df.insert(0, df[0])
# # df.insert(0, df[0])
# #添加最后一行元素
# df.append(df[-1])
# # df.append(df[-1])
# # df.append(df[-1])
# # df.append(df[-1])
# # df.append(df[-1])
# # df.append(df[-1])
# # df.append(df[-1])
# # df.append(df[-1])
# # df.append(df[-1])
# # df.append(df[-1])
# # df.append(df[-1])
# #print(df)
# # # 获取第一行数据
# #first_row = (df.iloc[0]).T
# #print(first_row)
# # # 重复第一行，若干次，比如重复 3 次
# # repeated_first_row = pd.concat([first_row] * 1, ignore_index=True)
# # # 在 DataFrame 开头添加重复的第一行
# # df = pd.concat([repeated_first_row, df], ignore_index=True)
# #print(df)
# df = pd.DataFrame(df)
#print(df)
df = pd.read_csv('/home/diamondlee/VKConTieta_ws/src/tieta_mpc_sim_demo/position_results/mpc_position_20240319_115105/mpc_all_joints_positions.csv')

df = df.iloc[:, 0:9]


# 计算速度，两两作差除以0.05
velocity = df.diff()/0.025
#print(velocity)

velocity = velocity.dropna(axis=0)
#print(velocity)

# # 添加一行零值
# zeros_row = pd.DataFrame([[0]*len(velocity.columns)], columns=velocity.columns)
# velocity = pd.concat([velocity,zeros_row], ignore_index=True)
# # 计算最后几个速度数据点的斜率
# last_few_velocities = velocity[-15:]  # 假设最后5个数据点
# slope = np.diff(last_few_velocities) / 0.05  # 假设时间间隔为0.05秒

# # 生成新的零速度数据点
# num_zeros = 15  # 新生成的零速度数据点数量
# new_velocities = []
# for i in range(1, num_zeros + 1):
#     new_velocity = last_few_velocities[-1] - slope[-1] * i
#     new_velocities.append(new_velocity)


# velocity.insert(0, [0]*len(velocity[0]))
# velocity.append([0]*len(velocity[0]))
# velocity = pd.DataFrame(velocity)
# print(velocity)
print(velocity)

# velocity = pd.DataFrame(velocity)
# # # # 在数据结构的开头和末尾分别插入全零行
# # # zero_row = pd.Series([0] * len(velocity.columns), index=velocity.columns)
# # # velocity = pd.concat([zero_row.to_frame().T, velocity, zero_row.to_frame().T], ignore_index=True)

# # print(velocity)

# #绘制速度曲线
# plt.figure(figsize=(10, 6))
# i=0
# for col in velocity.columns:
#     plt.plot(velocity[col], label=i)
#     i = i+1


# #! 现在你可以根据需要对 DataFrame 进行插值或补全，以使速度数据顺滑地趋向于零
# # 例如，使用插值函数进行线性插值
# smoothed_velocity = smoothed_velocity.interpolate(method='polynomial', order=25)
# print(smoothed_velocity.index)
# print(smoothed_velocity[0])
velocity_1 = velocity
point_end = -25
velocity_col = 0
df_end = pd.DataFrame({'time': range(56), 'value': [velocity.iloc[point_end -2,velocity_col],np.nan,velocity.iloc[point_end,velocity_col],np.nan,np.nan,velocity.iloc[point_end,velocity_col]*0.9,np.nan,np.nan,np.nan,np.nan, np.nan, np.nan,velocity.iloc[point_end,velocity_col]*0.75, np.nan, np.nan,np.nan,velocity.iloc[point_end,velocity_col]*0.5,np.nan, np.nan,velocity.iloc[point_end,velocity_col]*0.35, np.nan,np.nan, np.nan,np.nan,np.nan,velocity.iloc[point_end,velocity_col]*0.03,np.nan,np.nan,np.nan,np.nan,np.nan,np.nan,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]})

df_start = pd.DataFrame({'time': range(28), 'value': [velocity.iloc[4,velocity_col],velocity.iloc[3,velocity_col],velocity.iloc[2,velocity_col],velocity.iloc[1,velocity_col],velocity.iloc[0,velocity_col],np.nan,velocity.iloc[0,velocity_col]*0.8,np.nan,np.nan,np.nan,np.nan, np.nan, np.nan,np.nan,np.nan,np.nan,velocity.iloc[0,velocity_col]*0.20, np.nan, np.nan,np.nan,np.nan,np.nan,velocity.iloc[0,velocity_col]*0.03,np.nan, np.nan,velocity.iloc[0,velocity_col]*0.003, np.nan,0.0]})
print(velocity_col)
# 创建一个包含所有时间点的新 DataFrame
time_range_end = pd.DataFrame({'time': range(56)})
time_range_start = pd.DataFrame({'time': range(28)})

# 合并两个 DataFrame，使用 time 列作为连接键
merged_df_end = pd.merge(time_range_end, df_end, on='time', how='left')
merged_df_start = pd.merge(time_range_start, df_start, on='time', how='left')

# 对 value 列进行插值
interpolated_df_end = merged_df_end.interpolate(method='polynomial',order=3)
interpolated_df_start = merged_df_start.interpolate(method='polynomial',order=3)
print(2)

interpolated_df_start[interpolated_df_start.columns[::-1]] = interpolated_df_start[interpolated_df_start.columns[::-1]].values[::-1]
print(3)
#i
final_df_end = interpolated_df_end['value']
# print(final_df)
velocity = velocity.iloc[:(point_end -2),:]
print(velocity)
final_plot_df_end = pd.concat([velocity.iloc[:,velocity_col],final_df_end],ignore_index=True)
print(4)
print(final_plot_df_end)

#init_df = interpolated_df_start['value'].head(38)
init_df_start = interpolated_df_start['value'].iloc[0:23]

final_plot_df = pd.concat([init_df_start,final_plot_df_end],ignore_index=True)
print("dahdasdjkahdkahdkahda: ",len(final_plot_df))

smoothed_velocity = (final_plot_df.to_frame()).apply(ewma)
print("asfasfjakljflajflajflaj: ",len(smoothed_velocity))
new_timestep = (0.025 * len(velocity_1) + 0.025) / len(smoothed_velocity)

# 绘制平滑速度曲线
plt.figure(figsize=(15, 5))
plt.ylim(-0.3,0.8)
plt.xlim(-1,24)
x_tune = [0.0 + new_timestep*j for j in range(len(smoothed_velocity))]
x_ = [0 + 0.025*j for j in range(len(smoothed_velocity))]
i=0
# for col in smoothed_velocity.columns:
#     plt.plot(x_ , velocity[col], label=i)
#     i = i+1
print(smoothed_velocity)
plt.plot(x_tune, smoothed_velocity,color='black', label='smoothed_velocity',linewidth=2.5)
print(velocity.iloc[:,velocity_col])
# plt.plot([-23*0.025 + 0.025*j for j in range(len(final_plot_df))],final_plot_df)
# plt.scatter([0 + 0.05*j for j in range(len(interpolated_df_end['value']))],interpolated_df_end['value'])
print(interpolated_df_end)
print((velocity_1.iloc[:,velocity_col]).tail(20))
print("new time step: ", new_timestep)
print("total time is: ", 0.025*(len(velocity_1)+1))
#print(velocity_1.iloc[:,1])
# 添加标题和图例
plt.title('Velocity Curves')
plt.grid(True)
plt.legend()

# 显示图形
plt.show()
