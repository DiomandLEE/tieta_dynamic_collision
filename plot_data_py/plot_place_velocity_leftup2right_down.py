import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def ewma(column):
    return column.ewm(alpha=0.075).mean()

# 读取第一个CSV文件，提取前两列
df1 = pd.read_csv('/home/diamondlee/VKConTieta_ws/src/open_door_tieta_mpc/positon_results/mpc_position_20240321_114338/mpc_all_joints_positions.csv', header=None, index_col=False)
#取df1的前两列
df1 = df1.values.tolist()
df1 = [x[:2] for x in df1]
#print(df1)
# 读取第二个CSV文件，提取所有列
df2 = pd.read_csv('/home/diamondlee/VKConTieta_ws/src/open_door_tieta_mpc/positon_results/ik_position_20240321_114706/ik_theta_URjoints_positions.csv', header=None, index_col=False)
#print(df2)
df2 = df2.values.tolist()
# 合并两个DataFrame
#df = pd.concat([df1, df2], axis=1, ignore_index=True)
df = [x+y for x, y in zip(df1, df2)]
# df.insert(0, df[0])



#添加最后一行元素
#df.append(df[-1])

#print(df)
df = pd.DataFrame(df)
#print(df)

# 计算速度，两两作差除以0.05
velocity = df.diff()/0.036968838526912184
velocity_1 = df.diff()/0.05
#print(velocity)

velocity = velocity.dropna(axis=0)
velocity_1 = velocity_1.dropna(axis=0)

# 假设 df 是你的 DataFrame，速度列名为 'velocity'
# 速度单位应该是位移/时间间隔，因此位移就是速度 * 时间间隔
# 使用 pandas 的 cumsum() 方法计算累积位移
displacement = (velocity_1[3] * 0.05).cumsum()

# 最终的位移就是累积位移的最后一个值
total_displacement = displacement.iloc[-1]

print("Total displacement:", total_displacement)
#print(velocity)plt.scatter(np.arange(38,38+len(velocity[3])), velocity[3], color='blue', label='Original Velocity')
# plt.plot
# zeros_row = pd.DataFrame([[0]*len(velocity.columns)], columns=velocity.columns)
# velocity = pd.concat([velocity,zeros_row], ignore_index=True)
colors = ['blue','green', 'orange', 'purple','cyan', 'pink',  'brown', 'red', 'olive']
labels = ['X', 'Y', 'Theta','Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6']
# # print(velocity)
# print(velocity)
plt.figure(figsize=(8, 6.6))
plt.xlabel('Time (s)',fontweight='bold')
plt.ylabel('Magnitude',fontweight='bold')
plt.title('Place Action Velocitys Over Time',fontweight='bold')
x_ticks = [0, 2, 4, 6, 8, 10, 12, 13.0 ]
plt.xlim(0,13.0)
#y 间隔0.1 -0.5到0.5
y_ticks = [-0.5, -0.4, -0.3, -0.2, -0.1, 0, 0.1, 0.2, 0.3, 0.4, 0.5]
plt.ylim(-0.5, 0.5)
# plt.text(0.0, -0.5, f'({0.0}, {-0.5})', ha='right', va='bottom')
# plt.text(len(velocity[3]) * 0.05 + 0.05, 0.5, f'({len(velocity[3]) * 0.05 + 0.05}, {0.5})', ha='right', va='bottom')
# 设置x和y轴的刻度
# plt.xticks([0.0,2, 4, 6, 8, 10, 12, 13.1], [f'{0.0}', f'{13.1}'])
# plt.yticks([-0.5, 0.5], [f'{-0.5}', f'{0.5}'])
# 设置刻度位置
# x_ticks = [x_min + i * (x_max - x_min) / 4 for i in range(5)]
# y_ticks = [y_min + i * (y_max - y_min) / 4 for i in range(5)]

# 设置x和y轴的刻度
plt.xticks(x_ticks, [f'{tick:.1f}' for tick in x_ticks])
plt.yticks(y_ticks, [f'{tick:.1f}' for tick in y_ticks])

# 创建 DataFrame
for i in range(9):
    df = pd.DataFrame({'time': range(41), 'value': [velocity.iloc[-1,i],velocity.iloc[-1,i],velocity.iloc[-1,i], np.nan,np.nan,np.nan,np.nan, velocity.iloc[-1,i]*0.95,np.nan, np.nan, np.nan,np.nan,np.nan, np.nan, np.nan, np.nan,np.nan,np.nan,np.nan,np.nan, np.nan,np.nan,np.nan, np.nan,np.nan,velocity.iloc[-1,i]*0.20, np.nan, np.nan,np.nan,np.nan, np.nan,velocity.iloc[-1,i]*0.02,np.nan, np.nan,np.nan,np.nan,velocity.iloc[-1,i]*0.003, np.nan,np.nan,0.0,0.0]})

    df_start = pd.DataFrame({'time': range(41), 'value': [velocity.iloc[2,i],velocity.iloc[1,i],velocity.iloc[1,i], np.nan,np.nan,np.nan,np.nan, velocity.iloc[0,i]*0.45,np.nan, np.nan, np.nan,np.nan,np.nan, np.nan, np.nan, np.nan,np.nan,np.nan,np.nan,np.nan, np.nan,np.nan,np.nan, np.nan,np.nan,velocity.iloc[0,i]*0.08, np.nan, np.nan,np.nan,np.nan, np.nan,velocity.iloc[0,i]*0.01,np.nan, np.nan,np.nan,np.nan,velocity.iloc[0,i]*0.003, np.nan,np.nan,0.0,0.0]})


    # 创建一个包含所有时间点的新 DataFrame
    time_range = pd.DataFrame({'time': range(41)})
    time_range_start = pd.DataFrame({'time': range(41)})

    # 合并两个 DataFrame，使用 time 列作为连接键
    merged_df = pd.merge(time_range, df, on='time', how='left')
    merged_df_start = pd.merge(time_range_start, df_start, on='time', how='left')

    # 对 value 列进行插值
    interpolated_df = merged_df.interpolate(method='polynomial',order=3)
    interpolated_df_start = merged_df_start.interpolate(method='polynomial',order=3)

    interpolated_df_start[interpolated_df_start.columns[::-1]] = interpolated_df_start[interpolated_df_start.columns[::-1]].values[::-1]
#interpolated_df_start = interpolated_df_start.iloc[:,::-1]

# # 输出插值后的 DataFrame
# print("\n插值后的 DataFrame:")
# print(interpolated_df)


# # 输出插值后的 DataFrame
# print(interpolated_df)
    final_df = interpolated_df['value'].tail(38)
# print(final_df)

    final_plot_df = pd.concat([velocity[i],final_df],ignore_index=True)

    #init_df = interpolated_df_start['value'].head(38)
    init_df = interpolated_df_start['value'].iloc[8:38]

    final_plot_df_start = pd.concat([init_df,final_plot_df],ignore_index=True)
# print(final_plot_df)
# print(final_plot_df.index)
    final_plot_df.astype('float16')
#final_plot_df = final_plot_df.to_frame()
    smoothed_velocity = (final_plot_df.to_frame()).apply(ewma)
    smoothed_velocity_start = (final_plot_df_start.to_frame()).apply(ewma)


    df_long = pd.DataFrame({'time': range(23), 'value': [smoothed_velocity_start.iloc[-7,0],smoothed_velocity_start.iloc[-6,0],smoothed_velocity_start.iloc[-5,0],smoothed_velocity_start.iloc[-4,0],smoothed_velocity_start.iloc[-3,0],smoothed_velocity_start.iloc[-2,0],smoothed_velocity_start.iloc[-1,0], np.nan,np.nan,np.nan,np.nan,np.nan,np.nan,np.nan,np.nan,np.nan,np.nan,np.nan,np.nan,0.1*smoothed_velocity_start.iloc[-1,0],np.nan,0.0,0.0]})
    time_range_long = pd.DataFrame({'time': range(23)})

    # 合并两个 DataFrame，使用 time 列作为连接键
    merged_df_long = pd.merge(time_range_long, df_long, on='time', how='left')

    interpolated_df_long = merged_df_long.interpolate(method='polynomial',order=3)
    final_plot_df_long = pd.concat([smoothed_velocity_start,interpolated_df_long['value'].tail(16)],ignore_index=True)

    new_timestep = 0.05 * len(velocity[3]) / len(final_plot_df_long)

    x_long = [0+new_timestep*j for j in range(len(final_plot_df_long))]
    plt.plot(x_long,final_plot_df_long, color=colors[i], label=labels[i],linewidth=2.5)


# print(final_df)
# print(interpolated_df_start["value"])
# print(smoothed_velocity_start)
# print(final_plot_df_start)

# print(interpolated_df_long)

# print(len(velocity[3]))
# #打印final_plot_df_long的长度
# print(len(final_plot_df_long))
# 绘制平滑速度曲线
print(len(velocity[3]) * 0.05)
print(len(final_plot_df_long))
print(new_timestep)
# 假设 df 是你的 DataFrame，速度列名为 'velocity'
    # 速度单位应该是位移/时间间隔，因此位移就是速度 * 时间间隔
    # 使用 pandas 的 cumsum() 方法计算累积位移
displacement = (final_plot_df_long[0] * new_timestep).cumsum()

    # 最终的位移就是累积位移的最后一个值
total_displacement = displacement.iloc[-1]
print("Total displacement long:", total_displacement)
print(len(velocity[3]) * 0.05 + 0.05 - 8*new_timestep)

# # plt.plot(interpolated_df_start["value"])
# # plt.plot(velocity[3], color='red')
# x = [0+0.05*i for i in range(len(smoothed_velocity_start))]
# plt.scatter([0.0+0.05*i for i in range(len(velocity[5]))], velocity[5], color='blue', label='Original Velocity')
# # plt.plot(final_plot_df_start, color='red', label='lengethed Velocity')
# new_timestep = 0.05 * len(velocity[3]) / len(final_plot_df_start)
# print(new_timestep)

# # 假设 df 是你的 DataFrame，速度列名为 'velocity'
# # 速度单位应该是位移/时间间隔，因此位移就是速度 * 时间间隔
# # 使用 pandas 的 cumsum() 方法计算累积位移
# displacement = (final_plot_df_long[0] * new_timestep).cumsum()

# # 最终的位移就是累积位移的最后一个值
# total_displacement = displacement.iloc[-1]

# print("Total displacement long:", total_displacement)

# x_long = [0+new_timestep*i for i in range(len(final_plot_df_long))]
# #plt.plot(x,smoothed_velocity_start[0], color='green', label='Smoothed Velocity',linewidth=5)
# plt.plot(x_long,final_plot_df_long, color='blue', label='Smoothed-len Velocity')

plt.grid(True)


# 添加标题和图例
# plt.title('Velocity Curves')
plt.legend(loc='upper right')

# 显示图形
plt.show()