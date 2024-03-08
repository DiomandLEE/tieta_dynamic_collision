#ifndef parse_CSV_H
#define parse_CSV_H

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <math.h>
#include <map>
#include <sstream>
#include <unistd.h>
#include <boost/multiprecision/cpp_dec_float.hpp>
#include <iomanip>
#include <limits>
#include <ros/ros.h>
// include self-define msg
#include "DoorTrajPub/Angles.h"
#include "DoorTrajPub/AnglesList.h"

using namespace std;
using namespace boost::multiprecision;

std::vector<map<double, vector<double>>> parseCSV2Map(const std::string &file_in, int &_expand_n, bool &_delay_mode, int &_loopHz);

#endif  // MY_LIBRARY_H
