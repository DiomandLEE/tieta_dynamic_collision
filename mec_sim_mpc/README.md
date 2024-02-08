# mec_mpc_sim
更改了约束，加入了起始速度和位置约束，对于运动方程，也采用了积分运算得到  
运动学方程采用：在0.1时间内对速度进行积分，不是简单的离散化  
采用的mpc_result是第三个元素  
retimed_traj_pick_new是重新生成的轨迹。最大速度和角速度都是0.3/s
这里对速度的近似采取的是backward Euler method，可能会有一些问题
后续更改为forward Euler method
