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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/cmake-gui

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/vahid/My_WorkSpace/Project_Code_6_Qt4

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/vahid/My_WorkSpace/Project_Code_6_Qt4/build

# Include any dependencies generated for this target.
include CMakeFiles/Manage.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Manage.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Manage.dir/flags.make

CMakeFiles/Manage.dir/Manage.cc.o: CMakeFiles/Manage.dir/flags.make
CMakeFiles/Manage.dir/Manage.cc.o: ../Manage.cc
	$(CMAKE_COMMAND) -E cmake_progress_report /home/vahid/My_WorkSpace/Project_Code_6_Qt4/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/Manage.dir/Manage.cc.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/Manage.dir/Manage.cc.o -c /home/vahid/My_WorkSpace/Project_Code_6_Qt4/Manage.cc

CMakeFiles/Manage.dir/Manage.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Manage.dir/Manage.cc.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/vahid/My_WorkSpace/Project_Code_6_Qt4/Manage.cc > CMakeFiles/Manage.dir/Manage.cc.i

CMakeFiles/Manage.dir/Manage.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Manage.dir/Manage.cc.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/vahid/My_WorkSpace/Project_Code_6_Qt4/Manage.cc -o CMakeFiles/Manage.dir/Manage.cc.s

CMakeFiles/Manage.dir/Manage.cc.o.requires:
.PHONY : CMakeFiles/Manage.dir/Manage.cc.o.requires

CMakeFiles/Manage.dir/Manage.cc.o.provides: CMakeFiles/Manage.dir/Manage.cc.o.requires
	$(MAKE) -f CMakeFiles/Manage.dir/build.make CMakeFiles/Manage.dir/Manage.cc.o.provides.build
.PHONY : CMakeFiles/Manage.dir/Manage.cc.o.provides

CMakeFiles/Manage.dir/Manage.cc.o.provides.build: CMakeFiles/Manage.dir/Manage.cc.o

# Object files for target Manage
Manage_OBJECTS = \
"CMakeFiles/Manage.dir/Manage.cc.o"

# External object files for target Manage
Manage_EXTERNAL_OBJECTS =

libManage.a: CMakeFiles/Manage.dir/Manage.cc.o
libManage.a: CMakeFiles/Manage.dir/build.make
libManage.a: CMakeFiles/Manage.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX static library libManage.a"
	$(CMAKE_COMMAND) -P CMakeFiles/Manage.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Manage.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Manage.dir/build: libManage.a
.PHONY : CMakeFiles/Manage.dir/build

CMakeFiles/Manage.dir/requires: CMakeFiles/Manage.dir/Manage.cc.o.requires
.PHONY : CMakeFiles/Manage.dir/requires

CMakeFiles/Manage.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Manage.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Manage.dir/clean

CMakeFiles/Manage.dir/depend:
	cd /home/vahid/My_WorkSpace/Project_Code_6_Qt4/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/vahid/My_WorkSpace/Project_Code_6_Qt4 /home/vahid/My_WorkSpace/Project_Code_6_Qt4 /home/vahid/My_WorkSpace/Project_Code_6_Qt4/build /home/vahid/My_WorkSpace/Project_Code_6_Qt4/build /home/vahid/My_WorkSpace/Project_Code_6_Qt4/build/CMakeFiles/Manage.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Manage.dir/depend

