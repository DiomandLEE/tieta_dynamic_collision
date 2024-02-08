#include <parseCSV/parseCSV.h>

std::vector<map<double, vector<double>>> parseCSV2Map(const std::string &file_in, int& _expand_n,bool &_delay_mode,int &_loopHz) {
    ifstream fs;
    //debug
    cout << "file_in: " << file_in << endl;
    fs.open(file_in);
    if (!fs.is_open())
    {
        ROS_ERROR("Cannot open file: %s", file_in.c_str());
        return {{},{}};
    }
    string lineStr;
    map<double, vector<double>> desired_timedTraj;
    map<double, vector<double>> desired_AnglesList;
    vector<map<double, vector<double>>> desired_timedTotalTraj;
    double time = 0.0;
    int line = 0;
    //这个不需要了,因为考虑到,这样子做,csv中的line数会影响,能否取到最后一个点
    //int traj_sample = (1 / _loopHz) * 100;

    while (getline(fs, lineStr))
    {
        // 分割字符串,到时候把csv里的第一行去掉,因为第一行是英文介绍
        if(line > 0)
        {
            stringstream ss(lineStr);
            string str;
            vector<double> traj;
            vector<double> angles;
            int i = 0;
            while (getline(ss, str, ','))
            {
                //continue;
                //把CSV的第一行去掉
                //todo 这个是当时想要只做小车的x,y,theta，所以只取了前三个自由度。
                // 第一个不需要，第一个是时间的坐标
                //! 这部分截取用来给小车，发布小车的轨迹
                if(i >= 0 && i < 3)
                {
                    istringstream iss(str);
                    double str2double;
                    iss >> std::setprecision(std::numeric_limits<double>::max_digits10) >> str2double;
                    traj.push_back(str2double);
                }
            //for 7-dof arm , msg define considerless
            /*
            {
            if(i = 4)
            {
                istringstream iss1(str);
                double str2double1;
                iss1 >> std::setprecision(std::numeric_limits<double>::max_digits10) >> str2double1;
                angles.joint1 = str2double1;
            }
            if(i = 5)
            {
                istringstream iss2(str);
                double str2double2;
                iss2 >> std::setprecision(std::numeric_limits<double>::max_digits10) >> str2double2;
                angles.joint2 = str2double2;
            }
            if(i = 6)
            {
                istringstream iss3(str);
                double str2double3;
                iss3 >> std::setprecision(std::numeric_limits<double>::max_digits10) >> str2double3;
                angles.joint3 = str2double3;
            }
            if(i = 7)
            {
                istringstream iss4(str);
                double str2double4;
                iss4 >> std::setprecision(std::numeric_limits<double>::max_digits10) >> str2double4;
                angles.joint4 = str2double4;
            }
            if(i = 8)
            {
                istringstream iss5(str);
                double str2double5;
                iss5 >> std::setprecision(std::numeric_limits<double>::max_digits10) >> str2double5;
                angles.joint5 = str2double5;
            }
            if(i = 9)
            {
                istringstream iss6(str);
                double str2double6;
                iss6 >> std::setprecision(std::numeric_limits<double>::max_digits10) >> str2double6;
                angles.joint6 = str2double6;
            }
            if(i = 10)
            {
                istringstream iss7(str);
                double str2double7;
                iss7 >> std::setprecision(std::numeric_limits<double>::max_digits10) >> str2double7;
                angles.joint7 = str2double7;
            }
            }
            */
                if(i >= 3 && i <= 9)
                {
                    istringstream iss1(str);
                    double str2double1;
                    iss1 >> std::setprecision(std::numeric_limits<double>::max_digits10) >> str2double1;
                    //todo 解析的时候就要把弧度转为角度，因为kinova接收的是角度 deg //done
                    str2double1 = str2double1 * 180 / M_PI;
                    angles.push_back(str2double1);
                }

                i++;

                if(i == 10)
                {
                    break;
                }
                //csv中一行的分类完毕
            }
            if (traj.size() == 3 && angles.size() == 7)
            {
                // 存入map
                //获取轨迹部分
                //时间部分
                desired_timedTraj[time] = traj;
                desired_AnglesList[time] = angles;
                time += 0.1;
            }
        }
        line++;
    }

    //输出
    for (map<double, vector<double>>::iterator iter = desired_timedTraj.begin(); iter!=desired_timedTraj.end();iter++)
    {
        cout << "time is " << iter->first << " x is "<< iter->second[0] << " y is " << iter->second[1] << " theta is " << iter->second[2] << endl;
    }
    for(map<double, vector<double>>::iterator iter = desired_AnglesList.begin(); iter!=desired_AnglesList.end();iter++)
    {
        cout << "time is " << iter->first << " joint1 is "<< iter->second[0] << " joint2 is " << iter->second[1] << " joint3 is " << iter->second[2] << " joint4 is " << iter->second[3] << " joint5 is " << iter->second[4] << " joint6 is " << iter->second[5] << " joint7 is " << iter->second[6] << endl;
    }
    //对于底盘的轨迹点
    //! 这些都是为了保证mpc的进行，进行的后处理
    //为了使最后的速度为0，把track的轨迹延长
    //! 取map中的最后一个元素
    auto _last_map_factor_traj = desired_timedTraj.end();
    _last_map_factor_traj--;
    auto _last_Trajpoint = _last_map_factor_traj->second;
    //ipad中有推导为什么是_expand_n - 2

    //对于机械臂的轨迹点
    auto _last_map_factor_AnglesList = desired_AnglesList.end();
    _last_map_factor_AnglesList--;
    auto _last_Angles = _last_map_factor_AnglesList->second;

    for (int j = 0; j < _expand_n - 2; j++)
    {
        desired_timedTraj[time] = _last_Trajpoint;
        desired_AnglesList[time] = _last_Angles;
        time += 0.1;
    }
    //速度作为模型的输入时，需要在开头补齐一个delay_,这样的话前面补了一个，后面补了_mpc_step-2个，一共是_mpc_step - 1个
    //然而，真实的N是_mpc_step - 1个，这是因为在fg var中 “+ _mpc_step”导致的
    if(_delay_mode)
    {
        auto _mapfactor_traj_forDelay = desired_timedTraj.begin();
        auto _trajpoint_forDelay = _mapfactor_traj_forDelay->second;
        desired_timedTraj[-0.1] = _trajpoint_forDelay;

        auto _mapfactor_AnglesList_forDelay = desired_AnglesList.begin();
        auto _Angles_forDelay = _mapfactor_AnglesList_forDelay->second;
        desired_AnglesList[-0.1] = _Angles_forDelay;
    }
    desired_timedTotalTraj.push_back(desired_timedTraj);
    desired_timedTotalTraj.push_back(desired_AnglesList);

    return desired_timedTotalTraj;
}