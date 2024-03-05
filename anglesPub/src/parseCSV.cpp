#include <parseCSV/parseCSV.h>


std::vector<map<double, vector<double>>> parseCSV2Map(const std::string &file_in, int& _expand_n,bool &_delay_mode,int &_loopHz) {
    ifstream fs;
    fs.open(file_in);
    if (!fs.is_open())
    {
        ROS_ERROR("Cannot open file: %s", file_in.c_str());
        return {};
    }
    string lineStr;
    map<double, vector<double>> desired_AnglesList;
    vector<map<double, vector<double>>> desired_timedTotalTraj;
    double time = 0.00;
    int line = 0;
    std::cout << "start parse csv" << std::endl;
    int traj_sample = 1;

    while (getline(fs, lineStr))
    {
        // 分割字符串,到时候把csv里的第一行去掉,因为第一行是英文介绍
        if(line % traj_sample == 0)
        {
            std::cout << "line: " << line << std::endl;
            stringstream ss(lineStr);
            string str;
            vector<double> angles;
            int i = 0;
            while (getline(ss, str, ','))
            {
                //continue;
                //把CSV的第一行去掉
                //todo 这个是当时想要只做小车的x,y,theta，所以只取了前三个自由度。
                // 第一个不需要，第一个是时间的坐标
                //! 这部分截取用来给小车，发布小车的轨迹

                if(i > 0 && i <= 9)
                {
                    istringstream iss1(str);
                    double str2double1;
                    iss1 >> std::setprecision(std::numeric_limits<double>::max_digits10) >> str2double1;
                    //todo 解析的时候就要把弧度转为角度，因为kinova接收的是角度 deg //done
                    //str2double1 = str2double1
                    angles.push_back(str2double1);
                }

                i++;

                if(i == 10)
                {
                    break;
                }
                //csv中一行的分类完毕
            }
            if (angles.size() == 9)
            {
                // 存入map
                //获取轨迹部分
                //时间部分
                desired_AnglesList[time] = angles;
                time += 0.01;
            }
        }
        line++;
    }

    //输出
    // for (map<double, vector<double>>::iterator iter = desired_AnglesList.begin(); iter!=desired_AnglesList.end();iter++)
    // {
    //     cout << "time is " << iter->first << " date is "<< iter->second[0] << "," << iter->second[1] << "," << iter->second[2] <<
    //         "," << iter->second[3] << "," << iter->second[4] << "," << iter->second[5] <<
    //         " ," << iter->second[6] << "," << iter->second[7] << "," << iter->second[8] << endl;
    // }
    //对于底盘的轨迹点
    //! 这些都是为了保证mpc的进行，进行的后处理
    //为了使最后的速度为0，把track的轨迹延长
    //! 取map中的最后一个元素

    //对于机械臂的轨迹点
    auto _last_map_factor_AnglesList = desired_AnglesList.end();
    _last_map_factor_AnglesList--;
    auto _last_Angles = _last_map_factor_AnglesList->second;

    for (int j = 0; j < _expand_n - 2; j++)
    {
        desired_AnglesList[time] = _last_Angles;
        time += 0.01;
    }
    for (map<double, vector<double>>::iterator iter = desired_AnglesList.begin(); iter!=desired_AnglesList.end();iter++)
    {
        cout << "time is " << iter->first << " date is "<< iter->second[0] << "," << iter->second[1] << "," << iter->second[2] <<
            "," << iter->second[3] << "," << iter->second[4] << "," << iter->second[5] <<
            " ," << iter->second[6] << "," << iter->second[7] << "," << iter->second[8] << endl;
    }
    //速度作为模型的输入时，需要在开头补齐一个delay_,这样的话前面补了一个，后面补了_mpc_step-2个，一共是_mpc_step - 1个
    //然而，真实的N是_mpc_step - 1个，这是因为在fg var中 “+ _mpc_step”导致的
    if(_delay_mode)
    {
        auto _mapfactor_AnglesList_forDelay = desired_AnglesList.begin();
        auto _Angles_forDelay = _mapfactor_AnglesList_forDelay->second;
        desired_AnglesList[-0.01] = _Angles_forDelay;
    }
    desired_timedTotalTraj.push_back(desired_AnglesList);

    return desired_timedTotalTraj;
}