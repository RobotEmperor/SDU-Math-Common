# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/yik/sdu_ws/SDU-Math-Common

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/yik/sdu_ws/SDU-Math-Common/build

# Include any dependencies generated for this target.
include CMakeFiles/sdu_math.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/sdu_math.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/sdu_math.dir/flags.make

CMakeFiles/sdu_math.dir/sdu_math/src/kinematics.cpp.o: CMakeFiles/sdu_math.dir/flags.make
CMakeFiles/sdu_math.dir/sdu_math/src/kinematics.cpp.o: ../sdu_math/src/kinematics.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yik/sdu_ws/SDU-Math-Common/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/sdu_math.dir/sdu_math/src/kinematics.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sdu_math.dir/sdu_math/src/kinematics.cpp.o -c /home/yik/sdu_ws/SDU-Math-Common/sdu_math/src/kinematics.cpp

CMakeFiles/sdu_math.dir/sdu_math/src/kinematics.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sdu_math.dir/sdu_math/src/kinematics.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yik/sdu_ws/SDU-Math-Common/sdu_math/src/kinematics.cpp > CMakeFiles/sdu_math.dir/sdu_math/src/kinematics.cpp.i

CMakeFiles/sdu_math.dir/sdu_math/src/kinematics.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sdu_math.dir/sdu_math/src/kinematics.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yik/sdu_ws/SDU-Math-Common/sdu_math/src/kinematics.cpp -o CMakeFiles/sdu_math.dir/sdu_math/src/kinematics.cpp.s

CMakeFiles/sdu_math.dir/sdu_math/src/kinematics.cpp.o.requires:

.PHONY : CMakeFiles/sdu_math.dir/sdu_math/src/kinematics.cpp.o.requires

CMakeFiles/sdu_math.dir/sdu_math/src/kinematics.cpp.o.provides: CMakeFiles/sdu_math.dir/sdu_math/src/kinematics.cpp.o.requires
	$(MAKE) -f CMakeFiles/sdu_math.dir/build.make CMakeFiles/sdu_math.dir/sdu_math/src/kinematics.cpp.o.provides.build
.PHONY : CMakeFiles/sdu_math.dir/sdu_math/src/kinematics.cpp.o.provides

CMakeFiles/sdu_math.dir/sdu_math/src/kinematics.cpp.o.provides.build: CMakeFiles/sdu_math.dir/sdu_math/src/kinematics.cpp.o


CMakeFiles/sdu_math.dir/sdu_math/src/statics_math.cpp.o: CMakeFiles/sdu_math.dir/flags.make
CMakeFiles/sdu_math.dir/sdu_math/src/statics_math.cpp.o: ../sdu_math/src/statics_math.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yik/sdu_ws/SDU-Math-Common/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/sdu_math.dir/sdu_math/src/statics_math.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sdu_math.dir/sdu_math/src/statics_math.cpp.o -c /home/yik/sdu_ws/SDU-Math-Common/sdu_math/src/statics_math.cpp

CMakeFiles/sdu_math.dir/sdu_math/src/statics_math.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sdu_math.dir/sdu_math/src/statics_math.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yik/sdu_ws/SDU-Math-Common/sdu_math/src/statics_math.cpp > CMakeFiles/sdu_math.dir/sdu_math/src/statics_math.cpp.i

CMakeFiles/sdu_math.dir/sdu_math/src/statics_math.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sdu_math.dir/sdu_math/src/statics_math.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yik/sdu_ws/SDU-Math-Common/sdu_math/src/statics_math.cpp -o CMakeFiles/sdu_math.dir/sdu_math/src/statics_math.cpp.s

CMakeFiles/sdu_math.dir/sdu_math/src/statics_math.cpp.o.requires:

.PHONY : CMakeFiles/sdu_math.dir/sdu_math/src/statics_math.cpp.o.requires

CMakeFiles/sdu_math.dir/sdu_math/src/statics_math.cpp.o.provides: CMakeFiles/sdu_math.dir/sdu_math/src/statics_math.cpp.o.requires
	$(MAKE) -f CMakeFiles/sdu_math.dir/build.make CMakeFiles/sdu_math.dir/sdu_math/src/statics_math.cpp.o.provides.build
.PHONY : CMakeFiles/sdu_math.dir/sdu_math/src/statics_math.cpp.o.provides

CMakeFiles/sdu_math.dir/sdu_math/src/statics_math.cpp.o.provides.build: CMakeFiles/sdu_math.dir/sdu_math/src/statics_math.cpp.o


# Object files for target sdu_math
sdu_math_OBJECTS = \
"CMakeFiles/sdu_math.dir/sdu_math/src/kinematics.cpp.o" \
"CMakeFiles/sdu_math.dir/sdu_math/src/statics_math.cpp.o"

# External object files for target sdu_math
sdu_math_EXTERNAL_OBJECTS =

libsdu_math.so: CMakeFiles/sdu_math.dir/sdu_math/src/kinematics.cpp.o
libsdu_math.so: CMakeFiles/sdu_math.dir/sdu_math/src/statics_math.cpp.o
libsdu_math.so: CMakeFiles/sdu_math.dir/build.make
libsdu_math.so: CMakeFiles/sdu_math.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/yik/sdu_ws/SDU-Math-Common/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX shared library libsdu_math.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/sdu_math.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/sdu_math.dir/build: libsdu_math.so

.PHONY : CMakeFiles/sdu_math.dir/build

CMakeFiles/sdu_math.dir/requires: CMakeFiles/sdu_math.dir/sdu_math/src/kinematics.cpp.o.requires
CMakeFiles/sdu_math.dir/requires: CMakeFiles/sdu_math.dir/sdu_math/src/statics_math.cpp.o.requires

.PHONY : CMakeFiles/sdu_math.dir/requires

CMakeFiles/sdu_math.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/sdu_math.dir/cmake_clean.cmake
.PHONY : CMakeFiles/sdu_math.dir/clean

CMakeFiles/sdu_math.dir/depend:
	cd /home/yik/sdu_ws/SDU-Math-Common/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/yik/sdu_ws/SDU-Math-Common /home/yik/sdu_ws/SDU-Math-Common /home/yik/sdu_ws/SDU-Math-Common/build /home/yik/sdu_ws/SDU-Math-Common/build /home/yik/sdu_ws/SDU-Math-Common/build/CMakeFiles/sdu_math.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/sdu_math.dir/depend

