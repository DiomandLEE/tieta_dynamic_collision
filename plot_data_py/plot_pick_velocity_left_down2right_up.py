import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def ewma(column):
    return column.ewm(alpha=0.0475).mean()

df = pd.read_csv('/home/diamondlee/VKConTieta_ws/src/tieta_mpc_sim_demo/position_results/mpc_position_20240319_115105/mpc_all_joints_positions.csv')

df = df.iloc[:, 0:9]

# 绘制平滑速度曲线
plt.figure(figsize=(12, 5))
plt.xlabel('Time (s)',fontweight='bold')
plt.ylabel('Magnitude',fontweight='bold')
plt.title('Pick Action Velocitys Over Time',fontweight='bold')
# x_ticks = [0.0, 2.5, 5.0, 7.5 , 10.0, 12.5, 15, 17.5, 20.0, 22.5]
#间隔为3.0
x_ticks = [0.0, 3.0, 6.0, 9.0, 12.0, 15.0, 18.0, 21.0, 22.8]
plt.xlim(0.0,22.8)
#y 间隔0.1 -0.5到0.5
y_ticks = [-0.3, -0.2, -0.1, 0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
plt.ylim(-0.3, 0.7)
#plt.text(22.8,-0.3,'22.8',fontsize=10.5,color='black',ha='left',va='top')
# 设置x和y轴的刻度
plt.xticks(x_ticks, [f'{tick:.1f}' for tick in x_ticks])
plt.yticks(y_ticks, [f'{tick:.1f}' for tick in y_ticks])

colors = ['blue','green', 'orange', 'purple','cyan', 'pink',  'brown', 'red', 'olive']
labels = ['X', 'Y', 'Theta','Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6']

velocity = df.diff()/0.024
#print(velocity)

velocity = velocity.dropna(axis=0)
#print(velocity)
# #! 现在你可以根据需要对 DataFrame 进行插值或补全，以使速度数据顺滑地趋向于零
velocity_1 = velocity
point_end = -23
#velocity_col = 1

for velocity_col in range(9):
    print("point_end: ",point_end)
    df_end = pd.DataFrame({'time': range(56), 'value': [velocity_1.iloc[point_end -2,velocity_col],np.nan,velocity_1.iloc[point_end,velocity_col],np.nan,np.nan,velocity_1.iloc[point_end,velocity_col]*0.9,np.nan,np.nan,np.nan,np.nan, np.nan, np.nan,velocity_1.iloc[point_end,velocity_col]*0.75, np.nan, np.nan,np.nan,velocity_1.iloc[point_end,velocity_col]*0.5,np.nan, np.nan,velocity_1.iloc[point_end,velocity_col]*0.35, np.nan,np.nan, np.nan,np.nan,np.nan,velocity_1.iloc[point_end,velocity_col]*0.03,np.nan,np.nan,np.nan,np.nan,np.nan,np.nan,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]})

    df_start = pd.DataFrame({'time': range(28), 'value': [velocity_1.iloc[4,velocity_col],velocity_1.iloc[3,velocity_col],velocity_1.iloc[2,velocity_col],velocity_1.iloc[1,velocity_col],velocity_1.iloc[0,velocity_col],np.nan,velocity_1.iloc[0,velocity_col]*0.8,np.nan,np.nan,np.nan,np.nan, np.nan, np.nan,np.nan,np.nan,np.nan,velocity_1.iloc[0,velocity_col]*0.20, np.nan, np.nan,np.nan,np.nan,np.nan,velocity_1.iloc[0,velocity_col]*0.03,np.nan, np.nan,velocity_1.iloc[0,velocity_col]*0.003, np.nan,0.0]})
    #print(velocity_col)
    # 创建一个包含所有时间点的新 DataFrame
    time_range_end = pd.DataFrame({'time': range(56)})
    time_range_start = pd.DataFrame({'time': range(28)})

    # 合并两个 DataFrame，使用 time 列作为连接键
    merged_df_end = pd.merge(time_range_end, df_end, on='time', how='left')
    merged_df_start = pd.merge(time_range_start, df_start, on='time', how='left')

    # 对 value 列进行插值
    interpolated_df_end = merged_df_end.interpolate(method='polynomial',order=3)
    interpolated_df_start = merged_df_start.interpolate(method='polynomial',order=3)

    interpolated_df_start[interpolated_df_start.columns[::-1]] = interpolated_df_start[interpolated_df_start.columns[::-1]].values[::-1]
    # print(3)
    #i
    final_df_end = interpolated_df_end['value']
    # print(final_df)
    velocity = velocity_1.iloc[:(point_end -2),:]
    print("middle Veloity: ", len(velocity.iloc[:,velocity_col]))
    final_plot_df_end = pd.concat([velocity.iloc[:,velocity_col],final_df_end],ignore_index=True)
    # print(4)
    # print(final_plot_df_end)

    init_df_start = interpolated_df_start['value'].iloc[0:23]

    final_plot_df = pd.concat([init_df_start,final_plot_df_end],ignore_index=True)

    smoothed_velocity = (final_plot_df.to_frame()).apply(ewma)

    new_timestep = (0.025 * len(velocity_1) + 0.025) / len(smoothed_velocity)
    print("velocity_1: ", len(velocity_1))
    print("smoothed_velocity: ", len(smoothed_velocity))
    print("new time step: ", new_timestep)

    x_tune = [0.0 + new_timestep * j for j in range(len(smoothed_velocity))]
    # x_ = [0 + 0.025*j for j in range(len(smoothed_velocity))]
    # i=0
    # print(smoothed_velocity)
    plt.plot(x_tune, smoothed_velocity,color=colors[velocity_col], label=labels[velocity_col],linewidth=2.5)

# print(interpolated_df_end)
# print((velocity_1.iloc[:,velocity_col]).tail(20))

#print(velocity_1.iloc[:,1])
# 添加标题和图例
plt.grid(True)
plt.legend()

# 显示图形
plt.show()
