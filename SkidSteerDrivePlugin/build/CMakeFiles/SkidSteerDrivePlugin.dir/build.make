# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/vahid/My_WorkSpace/Project_Code_5_Qt4/SkidSteerDrivePlugin

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/vahid/My_WorkSpace/Project_Code_5_Qt4/SkidSteerDrivePlugin/build

# Include any dependencies generated for this target.
include CMakeFiles/SkidSteerDrivePlugin.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/SkidSteerDrivePlugin.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/SkidSteerDrivePlugin.dir/flags.make

CMakeFiles/SkidSteerDrivePlugin.dir/SkidSteerDrivePlugin.cc.o: CMakeFiles/SkidSteerDrivePlugin.dir/flags.make
CMakeFiles/SkidSteerDrivePlugin.dir/SkidSteerDrivePlugin.cc.o: ../SkidSteerDrivePlugin.cc
	$(CMAKE_COMMAND) -E cmake_progress_report /home/vahid/My_WorkSpace/Project_Code_5_Qt4/SkidSteerDrivePlugin/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/SkidSteerDrivePlugin.dir/SkidSteerDrivePlugin.cc.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/SkidSteerDrivePlugin.dir/SkidSteerDrivePlugin.cc.o -c /home/vahid/My_WorkSpace/Project_Code_5_Qt4/SkidSteerDrivePlugin/SkidSteerDrivePlugin.cc

CMakeFiles/SkidSteerDrivePlugin.dir/SkidSteerDrivePlugin.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/SkidSteerDrivePlugin.dir/SkidSteerDrivePlugin.cc.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/vahid/My_WorkSpace/Project_Code_5_Qt4/SkidSteerDrivePlugin/SkidSteerDrivePlugin.cc > CMakeFiles/SkidSteerDrivePlugin.dir/SkidSteerDrivePlugin.cc.i

CMakeFiles/SkidSteerDrivePlugin.dir/SkidSteerDrivePlugin.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/SkidSteerDrivePlugin.dir/SkidSteerDrivePlugin.cc.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/vahid/My_WorkSpace/Project_Code_5_Qt4/SkidSteerDrivePlugin/SkidSteerDrivePlugin.cc -o CMakeFiles/SkidSteerDrivePlugin.dir/SkidSteerDrivePlugin.cc.s

CMakeFiles/SkidSteerDrivePlugin.dir/SkidSteerDrivePlugin.cc.o.requires:
.PHONY : CMakeFiles/SkidSteerDrivePlugin.dir/SkidSteerDrivePlugin.cc.o.requires

CMakeFiles/SkidSteerDrivePlugin.dir/SkidSteerDrivePlugin.cc.o.provides: CMakeFiles/SkidSteerDrivePlugin.dir/SkidSteerDrivePlugin.cc.o.requires
	$(MAKE) -f CMakeFiles/SkidSteerDrivePlugin.dir/build.make CMakeFiles/SkidSteerDrivePlugin.dir/SkidSteerDrivePlugin.cc.o.provides.build
.PHONY : CMakeFiles/SkidSteerDrivePlugin.dir/SkidSteerDrivePlugin.cc.o.provides

CMakeFiles/SkidSteerDrivePlugin.dir/SkidSteerDrivePlugin.cc.o.provides.build: CMakeFiles/SkidSteerDrivePlugin.dir/SkidSteerDrivePlugin.cc.o

# Object files for target SkidSteerDrivePlugin
SkidSteerDrivePlugin_OBJECTS = \
"CMakeFiles/SkidSteerDrivePlugin.dir/SkidSteerDrivePlugin.cc.o"

# External object files for target SkidSteerDrivePlugin
SkidSteerDrivePlugin_EXTERNAL_OBJECTS =

libSkidSteerDrivePlugin.so: CMakeFiles/SkidSteerDrivePlugin.dir/SkidSteerDrivePlugin.cc.o
libSkidSteerDrivePlugin.so: CMakeFiles/SkidSteerDrivePlugin.dir/build.make
libSkidSteerDrivePlugin.so: CMakeFiles/SkidSteerDrivePlugin.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX shared library libSkidSteerDrivePlugin.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/SkidSteerDrivePlugin.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/SkidSteerDrivePlugin.dir/build: libSkidSteerDrivePlugin.so
.PHONY : CMakeFiles/SkidSteerDrivePlugin.dir/build

CMakeFiles/SkidSteerDrivePlugin.dir/requires: CMakeFiles/SkidSteerDrivePlugin.dir/SkidSteerDrivePlugin.cc.o.requires
.PHONY : CMakeFiles/SkidSteerDrivePlugin.dir/requires

CMakeFiles/SkidSteerDrivePlugin.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/SkidSteerDrivePlugin.dir/cmake_clean.cmake
.PHONY : CMakeFiles/SkidSteerDrivePlugin.dir/clean

CMakeFiles/SkidSteerDrivePlugin.dir/depend:
	cd /home/vahid/My_WorkSpace/Project_Code_5_Qt4/SkidSteerDrivePlugin/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/vahid/My_WorkSpace/Project_Code_5_Qt4/SkidSteerDrivePlugin /home/vahid/My_WorkSpace/Project_Code_5_Qt4/SkidSteerDrivePlugin /home/vahid/My_WorkSpace/Project_Code_5_Qt4/SkidSteerDrivePlugin/build /home/vahid/My_WorkSpace/Project_Code_5_Qt4/SkidSteerDrivePlugin/build /home/vahid/My_WorkSpace/Project_Code_5_Qt4/SkidSteerDrivePlugin/build/CMakeFiles/SkidSteerDrivePlugin.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/SkidSteerDrivePlugin.dir/depend

