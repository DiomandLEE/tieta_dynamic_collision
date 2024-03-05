#! /usr/bin/env python
# -*- coding: utf-8 -*
import moveit_commander
from geometry_msgs.msg import Point
import rospy
#import yaml
import csv
import os
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotTrajectory, RobotState
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Header

from geometry_msgs.msg import TransformStamped, Vector3Stamped
from itertools import chain
from math import pi

#fast speed
acc_scale = 0.15
velo_scale = 0.5
acc_scale_pick = 0.23
velo_scale_pick = 0.8
#medium speed
# acc_scale = 0.05
# velo_scale = 0.4
# acc_scale_pick = 0.2
# velo_scale_pick = 0.4
pickpath = "/home/diamondlee/VKConTieta_ws/src/tieta_env_P/tieta_env_PickAction10_08_14.csv"
path = "/home/ridgeback/catkin_ws/src/ridgeback_waypoints_execution/files/open_door_pull_0_new.csv"
open_door_path = "/home/ridgeback/catkin_ws/src/ridgeback_waypoints_execution/files/open_door_pull_1_new.csv"
pickpath_pre = '/home/ridgeback/catkin_ws/src/ridgeback_waypoints_execution/files/tieta_env_PickAction'
placepath_pre = '/home/ridgeback/catkin_ws/src/ridgeback_waypoints_execution/files/tieta_env_PlaceAction'

def generate_mock_state(commander):
    state = RobotState()
    state.is_diff = False
    state.joint_state.name = commander.get_active_joints()
    state.joint_state.position = [0] * 9
    return state# -*- coding: utf-8 -*

def points_transform(point):
    #! 这个没有必要，本来这么做的原因应该是，vkc的csv结果，可能与move_group的active_joint的name对不上
    '''
    new_x --> -y
    new_y : x
    theta  --> -pi/2

    changed to:
    new_x --> -x
    new_y : -y
    theta  --> -theta
    '''

    # new_point = [point[1],-point[0],point[2]-pi/2]
    # for _ in point[3:10]:
    #     new_point.append(_)

    new_point = [point[0], point[1], point[2]]
    for _ in point[3:10]:
        new_point.append(_)
    #! python中这是索引3-9，即第11个之前的元素在new_ppoint中
    return new_point

def main():
    rospy.init_node('moveit_time_parameter')
    pick_raw_traj = []
    #place_raw_traj = [] #pick和place分开
    #seq_len = 4 #这里不需要，我们每次只时间参数化一个csv

    csv_lines = []
    #!只读模式打开
    with open(pickpath, 'r') as f:
        #! 创建一个csv.reader对象，读取csv的对象
        jiaoreader = csv.reader(f)
        for row in jiaoreader:
            csv_lines.append(row) #!将每一行作为列表的元素，放入列表中，元素是每一行的字符串形式

    # csv_lines = [
    # ['1', '2', '3', '4', '5'],
    # ['6', '7', '8', '9', '10'],
    # ['11', '12', '13', '14', '15']
    #]

    #! 从第二行开始，将字符串转换为float，因为第一行是joint-name
    approach = list(map(lambda x: [float(p) for p in x], csv_lines[1:]))
        # approach.reverse()
    temp_a  =[]
    for t in approach:
        temp_a.append(points_transform(t))
    approach = temp_a
    pick_raw_traj.append(approach) #这就是一个二维列表，每一行都是csv中每一行的数据

    print(approach)
    print(len(approach))

    #!######################################################
    # csv_lines = []  # 创建一个空列表，用来存储从 CSV 文件中读取的行数据

    # pickpath = pickpath_pre + str(i) + '.csv'  # 创建一个文件路径，根据 pickpath_pre 和 i 的值拼接而成
    # # 打开 pickpath 对应的文件，使用 'r' 模式表示只读模式，并将文件对象赋值给变量 f
    # with open(pickpath, 'r') as f:
    #     jiaoreader = csv.reader(f)  # 创建一个 csv.reader 对象 jiaoreader，用来读取 CSV 文件 f 的内容
    # # 遍历 jiaoreader 对象，每次循环将读取的行数据添加到 csv_lines 列表中
    #     for row in jiaoreader:
    #         csv_lines.append(row)

    # # 将 csv_lines 列表中的每个元素（每个元素是一个字符串列表）转换为浮点数，并将结果保存到 approach 列表中
    # approach = list(map(lambda x: [float(p) for p in x], csv_lines[1:]))
        #?####################################
        # def str_to_float_list(str_list):
        # float_list = []
        # for str_num in str_list:
        #     float_list.append(float(str_num))
        # return float_list

        #approach = list(map(str_to_float_list, csv_lines[1:]))
        # 在这个示例中，我们定义了一个名为str_to_float_list的函数，它接受一个字符串列表作为输入，并返回一个将字符串转换为浮点数后的列表。
        # 然后我们使用map函数将str_to_float_list函数应用到csv_lines[1:]中的每个元素上，从而得到approach。
        # 结果与之前的lambda表达式相同，但更易于理解和调试。

        # map是Python内置的函数，它的作用是将一个函数应用于迭代器的每个元素，并返回一个新的迭代器，其中每个元素都是原迭代器中对应元素经过函数处理后的结果。
        # 在我们的例子中，map函数应用于csv_lines[1:]，它是一个二维列表，即每个元素也是一个列表，表示CSV文件中的一行数据。
        # 函数str_to_float_list应用于csv_lines[1:]的每个元素，即每行数据。str_to_float_list函数的作用是将每行数据中的字符串转换为浮点数，
        # 并返回一个包含浮点数的列表。map函数返回的结果approach是一个列表，其中每个元素是str_to_float_list函数应用于csv_lines[1:]中对应元素
        # （即每行数据）的结果。

        #approach = [str_to_float_list(row) for row in csv_lines[1:]]
        # 这个列表推导与之前的map函数产生的approach是等价的。它会遍历csv_lines[1:]中的每个元素（即每行数据），并将str_to_float_list函数应用于每个元素。
        # 结果是一个新的列表，其中每个元素都是str_to_float_list函数应用于csv_lines[1:]中对应元素（即每行数据）的结果。
        #?########################################

    # # 将 approach 列表中的每个元素（每个元素是一个浮点数列表）都传递给 points_transform 函数，并将结果保存到 temp_a 列表中
    # temp_a  =[]
    # for t in approach:
    #     temp_a.append(points_transform(t))

    # # 将 temp_a 列表添加到 pick_raw_traj 列表中
    # pick_raw_traj.append(temp_a)
    #!##########################################################

    ## print(pick_raw_traj)
    ## ##
    ## pick_raw_traj.reverse()
    ## ##
    ## print(pick_raw_traj)

    # for i in range(seq_len):
    #     csv_lines = []
    #     placepath = placepath_pre+str(i)+'.csv'
    #     with open(placepath, 'r') as f:
    #         reader = csv.reader(f)
    #         for row in reader:
    #             csv_lines.append(row)

    #     open_door = list(map(lambda x: [float(p) for p in x], csv_lines[1:]))
    #     # open_door.reverse()
    #     temp_a  =[]
    #     for t in open_door:
    #         temp_a.append(points_transform(t))
    #     open_door = temp_a
    #     place_raw_traj.append(open_door)

    ###
    ## place_raw_traj.reverse()
    ###

    #file_path_pick_pre = os.path.join(os.path.expanduser('~'), 'catkin_ws','intermediate_data', 'retimed_vkc_pick')
    #file_path_place_pre = os.path.join(os.path.expanduser('~'), 'catkin_ws', 'intermediate_data','retimed_vkc_place')

    # 这段代码首先导入了Python的os模块，然后使用os.path.expanduser('~')函数获取当前用户的home目录，然后将这个目录和其它字符串拼接在一起，
    # 形成一个文件路径。这个文件路径用于指向一个名为‘catkin_ws/intermediate_data/retimed_vkc_pick’的文件夹。这个文件夹是一个中间数据文件夹，
    # 用于存储一些中间数据。具体来说，这个文件路径由三个部分构成：
    # os.path.expanduser('~')函数返回当前用户的home目录，例如‘/home/user’。
    # ‘catkin_ws/intermediate_data/retimed_vkc_pick’是一个字符串，它是文件路径的中间部分。
    # os.path.join()函数用于将这三个部分拼接在一起，形成一个完整的文件路径。
    # 总的来说，这段代码的作用是创建一个文件路径，用于指向一个名为‘catkin_ws/intermediate_data/retimed_vkc_pick’的文件夹。


    vkc_group = moveit_commander.MoveGroupCommander('vkc_right_arm')  # type: moveit_commander.MoveGroupCommander.MoveGroupCommander
    print(pick_raw_traj[0]) #! 明白了这实际上就一行，它的size是1

    for i in range(len(pick_raw_traj)):
        #pick中的行数
        traj = RobotTrajectory()
        traj.joint_trajectory.joint_names = vkc_group.get_active_joints()
        traj.joint_trajectory.header = Header(stamp=rospy.Time.now())
        start = 0
        for t in pick_raw_traj[i]:
            # 遍历第i行中的所有元素
            p = JointTrajectoryPoint()
            p.positions = t
            p.time_from_start = rospy.Duration(start)
            #print(p)
            traj.joint_trajectory.points.append(p)
            start += 1
    #     print('Progressing:',i)
    #! 运行的的时候 需要source到moveit的源码安装，进行python
    retimed_traj = vkc_group.retime_trajectory(
            generate_mock_state(vkc_group),
            traj,
            algorithm='time_optimal_trajectory_generation',
            velocity_scaling_factor=velo_scale_pick,acceleration_scaling_factor=acc_scale_pick) #0.2

    # retimed_traj_1 = vkc_group.retime_trajectory(
    #         generate_mock_state(vkc_group),
    #         retimed_traj,
    #         algorithm='time_optimal_trajectory_generation',
    #         velocity_scaling_factor=velo_scale_pick,acceleration_scaling_factor=acc_scale_pick)

    # print(retimed_traj)
    # print(len(retimed_traj.joint_trajectory.points))

    # time_start = 0

    # traj = RobotTrajectory()
    # traj.joint_trajectory.joint_names = vkc_group.get_active_joints()
    # traj.joint_trajectory.header = Header(stamp=rospy.Time.now())

    # for i in range(len(retimed_traj.joint_trajectory.points)):
    #     # 遍历第i行中的所有元素
    #     p = retimed_traj.joint_trajectory.points[i]
    #     p.time_from_start = rospy.Duration(time_start)
    #     traj.joint_trajectory.points.append(p)
    #     time_start += 1
    #     print(p)

    # retimed_traj_1 = vkc_group.retime_trajectory(
    #         generate_mock_state(vkc_group),
    #         traj,
    #         algorithm='time_optimal_trajectory_generation',
    #         velocity_scaling_factor=velo_scale_pick,acceleration_scaling_factor=acc_scale_pick)
    # print(len(retimed_traj_1.joint_trajectory.points))
    # 创建一个名为 'retimed_traj.csv' 的 CSV 文件
    file_path_place = "/home/diamondlee/VKConTieta_ws/src/tieta_env_P/retimed_traj.csv"

    # 打开 'retimed_traj.csv' 文件，使用 'w' 模式表示写入模式，并将文件对象赋值给变量 file_save
    with open(file_path_place, 'w', newline='') as file_save:
        # 创建一个 csv.writer 对象 writer，用来写入 CSV 文件 file_save 的内容
        writer = csv.writer(file_save)
        # 将 retimed_traj 中的每个元素（每个元素是一个列表）写入到 CSV 文件中
        for row in retimed_traj.joint_trajectory.points:
            writer.writerow(row)



        # print(file_path)
        # file_path_pick = file_path_pick_pre + str(i)+ '.yaml'
        # with open(file_path_pick, 'w') as file_save:
        #     yaml.dump(retimed_traj, file_save, default_flow_style=True)

#         # 将文件路径按照特定的格式拼接
#        file_path_place = file_path_place_pre + str(i) + '.yaml'

#         # 打开文件路径对应的文件，使用 'w' 模式表示写入模式，并将文件对象赋值给变量 file_save
#           with open(file_path_place, 'w') as file_save:
#           # 将重新计时后的轨迹数据写入文件
#           # yaml.dump 函数将 retimed_traj 对象写入文件中，default_flow_style=True 表示以默认的流格式写入
#           yaml.dump(retimed_traj, file_save, default_flow_style=True)
# 这段代码的作用是将一个对象（retimed_traj）写入到一个 YAML 文件中。为了实现这个目的，首先需要构建一个文件路径（file_path_place），
# 然后使用 open 函数打开这个文件路径对应的文件。打开文件后，可以使用 yaml.dump 函数将对象写入到文件中。yaml.dump 函数的第二个参数是文件对象，
# 第一个参数是要写入的对象，default_flow_style=True 表示以默认的流格式写入。当对象被写入到文件中后，文件对象被关闭，这是由 with open 语句中的语法决定的。

# with open(file_path_place, 'w') as file_save: 是一个常用的写入文件的固定用法，其中 with 语句在 Python 中被称为上下文管理器，
# 它的作用是打开文件后自动关闭文件，以防止资源泄漏。
# 具体来说，with 语句会在代码块执行完毕后自动调用 file_save.close() 方法，因此不需要显式地调用 file_save.close() 来关闭文件。
# 这样可以确保文件在不再需要时被正确地关闭，避免了资源泄漏的问题。
# 另外，yaml.dump 函数用于将对象写入到文件中，其中 retimed_traj 是要写入的对象，file_save 是要写入的文件对象，
# default_flow_style=True 表示以默认的流格式写入。
# 总的来说，这段代码是一个常用的写入文件的固定用法，通过 with open 和 yaml.dump 函数可以方便地将对象写入到文件中。

# 如果你想将 retimed_traj 对象保存到其他文件中，你可以修改 file_path_place 的值，将它设为其他文件的路径，
# 然后再使用 yaml.dump() 函数将对象写入到这个文件中。
# 例如，如果你想将 retimed_traj 对象保存到一个名为 other_file.yaml 的文件中，你可以这样做：
# file_path_place = "other_file.yaml"

# with open(file_path_place, 'w') as file_save:
#     yaml.dump(retimed_traj, file_save, default_flow_style=True)


    # for i in range(len(place_raw_traj)):
    #     traj = RobotTrajectory()
    #     traj.joint_trajectory.joint_names = vkc_group.get_active_joints()
    #     traj.joint_trajectory.header = Header(stamp=rospy.Time.now())

    #     start = 0
    #     if i == 7 :

    #         for t in place_raw_traj[i]:
    #             # place_raw_traj[i].reverse()
    #             p = JointTrajectoryPoint()
    #             p.positions = t
    #             p.time_from_start = rospy.Duration(start)
    #             # print(p)
    #             traj.joint_trajectory.points.append(p)
    #             start += 1
    #             retimed_traj = vkc_group.retime_trajectory(
    #             generate_mock_state(vkc_group),
    #             traj,
    #             algorithm='time_optimal_trajectory_generation',
    #             velocity_scaling_factor=0.8,acceleration_scaling_factor=0.2)#0.2
    #     else:
    #         for t in place_raw_traj[i]:
    #             # place_raw_traj[i].reverse()
    #             p = JointTrajectoryPoint()
    #             p.positions = t
    #             p.time_from_start = rospy.Duration(start)
    #             # print(p)
    #             traj.joint_trajectory.points.append(p)
    #             start += 1
    #             retimed_traj = vkc_group.retime_trajectory(
    #             generate_mock_state(vkc_group),
    #             traj,
    #             algorithm='time_optimal_trajectory_generation',
    #             velocity_scaling_factor=velo_scale,acceleration_scaling_factor=acc_scale)#0.2
    #     print('Progressing:',i)
    #     file_path_place = file_path_place_pre + str(i) + '.yaml'
    #     with open(file_path_place, 'w') as file_save:
    #         yaml.dump(retimed_traj, file_save, default_flow_style=True)

if __name__ == "__main__":
    main()