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
CMAKE_SOURCE_DIR = /home/josel/robocomp/components/beta-robotica-class/Controller

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/josel/robocomp/components/beta-robotica-class/Controller/build

# Utility rule file for MyFirstComp_autogen.

# Include the progress variables for this target.
include src/CMakeFiles/MyFirstComp_autogen.dir/progress.make

src/CMakeFiles/MyFirstComp_autogen:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/josel/robocomp/components/beta-robotica-class/Controller/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Automatic MOC and UIC for target MyFirstComp"
	cd /home/josel/robocomp/components/beta-robotica-class/Controller/build/src && /usr/bin/cmake -E cmake_autogen /home/josel/robocomp/components/beta-robotica-class/Controller/build/src/CMakeFiles/MyFirstComp_autogen.dir/AutogenInfo.json ""

MyFirstComp_autogen: src/CMakeFiles/MyFirstComp_autogen
MyFirstComp_autogen: src/CMakeFiles/MyFirstComp_autogen.dir/build.make

.PHONY : MyFirstComp_autogen

# Rule to build all files generated by this target.
src/CMakeFiles/MyFirstComp_autogen.dir/build: MyFirstComp_autogen

.PHONY : src/CMakeFiles/MyFirstComp_autogen.dir/build

src/CMakeFiles/MyFirstComp_autogen.dir/clean:
	cd /home/josel/robocomp/components/beta-robotica-class/Controller/build/src && $(CMAKE_COMMAND) -P CMakeFiles/MyFirstComp_autogen.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/MyFirstComp_autogen.dir/clean

src/CMakeFiles/MyFirstComp_autogen.dir/depend:
	cd /home/josel/robocomp/components/beta-robotica-class/Controller/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/josel/robocomp/components/beta-robotica-class/Controller /home/josel/robocomp/components/beta-robotica-class/Controller/src /home/josel/robocomp/components/beta-robotica-class/Controller/build /home/josel/robocomp/components/beta-robotica-class/Controller/build/src /home/josel/robocomp/components/beta-robotica-class/Controller/build/src/CMakeFiles/MyFirstComp_autogen.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/MyFirstComp_autogen.dir/depend

