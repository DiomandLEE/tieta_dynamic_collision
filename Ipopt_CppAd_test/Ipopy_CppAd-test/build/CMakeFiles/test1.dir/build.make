# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/diamondlee/Controll_mac_test/Ipopt_CppAd_test/Ipopy_CppAd-test

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/diamondlee/Controll_mac_test/Ipopt_CppAd_test/Ipopy_CppAd-test/build

# Include any dependencies generated for this target.
include CMakeFiles/test1.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/test1.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/test1.dir/flags.make

CMakeFiles/test1.dir/cppad_test_track.cpp.o: CMakeFiles/test1.dir/flags.make
CMakeFiles/test1.dir/cppad_test_track.cpp.o: ../cppad_test_track.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/diamondlee/Controll_mac_test/Ipopt_CppAd_test/Ipopy_CppAd-test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/test1.dir/cppad_test_track.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test1.dir/cppad_test_track.cpp.o -c /home/diamondlee/Controll_mac_test/Ipopt_CppAd_test/Ipopy_CppAd-test/cppad_test_track.cpp

CMakeFiles/test1.dir/cppad_test_track.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test1.dir/cppad_test_track.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/diamondlee/Controll_mac_test/Ipopt_CppAd_test/Ipopy_CppAd-test/cppad_test_track.cpp > CMakeFiles/test1.dir/cppad_test_track.cpp.i

CMakeFiles/test1.dir/cppad_test_track.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test1.dir/cppad_test_track.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/diamondlee/Controll_mac_test/Ipopt_CppAd_test/Ipopy_CppAd-test/cppad_test_track.cpp -o CMakeFiles/test1.dir/cppad_test_track.cpp.s

# Object files for target test1
test1_OBJECTS = \
"CMakeFiles/test1.dir/cppad_test_track.cpp.o"

# External object files for target test1
test1_EXTERNAL_OBJECTS =

test1: CMakeFiles/test1.dir/cppad_test_track.cpp.o
test1: CMakeFiles/test1.dir/build.make
test1: CMakeFiles/test1.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/diamondlee/Controll_mac_test/Ipopt_CppAd_test/Ipopy_CppAd-test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable test1"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test1.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/test1.dir/build: test1

.PHONY : CMakeFiles/test1.dir/build

CMakeFiles/test1.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/test1.dir/cmake_clean.cmake
.PHONY : CMakeFiles/test1.dir/clean

CMakeFiles/test1.dir/depend:
	cd /home/diamondlee/Controll_mac_test/Ipopt_CppAd_test/Ipopy_CppAd-test/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/diamondlee/Controll_mac_test/Ipopt_CppAd_test/Ipopy_CppAd-test /home/diamondlee/Controll_mac_test/Ipopt_CppAd_test/Ipopy_CppAd-test /home/diamondlee/Controll_mac_test/Ipopt_CppAd_test/Ipopy_CppAd-test/build /home/diamondlee/Controll_mac_test/Ipopt_CppAd_test/Ipopy_CppAd-test/build /home/diamondlee/Controll_mac_test/Ipopt_CppAd_test/Ipopy_CppAd-test/build/CMakeFiles/test1.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/test1.dir/depend

