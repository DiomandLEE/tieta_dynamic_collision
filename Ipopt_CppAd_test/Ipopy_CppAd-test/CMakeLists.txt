cmake_minimum_required( VERSION 2.8 )
project(test)

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-O3" )
set( CMAKE_CXX_FLAGS "-std=c++11")

# 添加头文件 #给到的是cpp中的include
include_directories("/usr/local/include")
# 添加库 #给到的是make编译时的库
link_directories( /usr/local/lib )
# 添加需要编译的可执行文件
add_executable(test cppad_ipopt_demo.cpp)
add_executable(test_track cppad_test_track.cpp)
add_executable(test_no_retape cppad_test_obs_no_retape.cpp)
add_executable(test_retape cppad_test_obs_retape.cpp)
add_executable(test_fk urdf2casadi_test.cpp)
add_executable(test_fk_cppad urdf2casadi_cppad.cpp)
add_executable(test_fg_casadi urdf2casadi_track.cpp)
# 链接到对应的库
target_link_libraries(${PROJECT_NAME} ipopt)
target_link_libraries(test_track ipopt)
target_link_libraries(test_no_retape ipopt)
target_link_libraries(test_retape ipopt)
target_link_libraries(test_fk ipopt)
target_link_libraries(test_fk_cppad ipopt)
target_link_libraries(test_fg_casadi ipopt)


