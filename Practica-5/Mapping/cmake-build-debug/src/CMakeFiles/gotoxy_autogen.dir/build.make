# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.20

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /snap/clion/169/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /snap/clion/169/bin/cmake/linux/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/alumno/Documentos/g8-M1/Robotica-2021-2022/Practica-5/Mapping

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/alumno/Documentos/g8-M1/Robotica-2021-2022/Practica-5/Mapping/cmake-build-debug

# Utility rule file for gotoxy_autogen.

# Include any custom commands dependencies for this target.
include src/CMakeFiles/gotoxy_autogen.dir/compiler_depend.make

# Include the progress variables for this target.
include src/CMakeFiles/gotoxy_autogen.dir/progress.make

src/CMakeFiles/gotoxy_autogen:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/alumno/Documentos/g8-M1/Robotica-2021-2022/Practica-5/Mapping/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Automatic MOC and UIC for target gotoxy"
	cd /home/alumno/Documentos/g8-M1/Robotica-2021-2022/Practica-5/Mapping/cmake-build-debug/src && /snap/clion/169/bin/cmake/linux/bin/cmake -E cmake_autogen /home/alumno/Documentos/g8-M1/Robotica-2021-2022/Practica-5/Mapping/cmake-build-debug/src/CMakeFiles/gotoxy_autogen.dir/AutogenInfo.json Debug

gotoxy_autogen: src/CMakeFiles/gotoxy_autogen
gotoxy_autogen: src/CMakeFiles/gotoxy_autogen.dir/build.make
.PHONY : gotoxy_autogen

# Rule to build all files generated by this target.
src/CMakeFiles/gotoxy_autogen.dir/build: gotoxy_autogen
.PHONY : src/CMakeFiles/gotoxy_autogen.dir/build

src/CMakeFiles/gotoxy_autogen.dir/clean:
	cd /home/alumno/Documentos/g8-M1/Robotica-2021-2022/Practica-5/Mapping/cmake-build-debug/src && $(CMAKE_COMMAND) -P CMakeFiles/gotoxy_autogen.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/gotoxy_autogen.dir/clean

src/CMakeFiles/gotoxy_autogen.dir/depend:
	cd /home/alumno/Documentos/g8-M1/Robotica-2021-2022/Practica-5/Mapping/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/alumno/Documentos/g8-M1/Robotica-2021-2022/Practica-5/Mapping /home/alumno/Documentos/g8-M1/Robotica-2021-2022/Practica-5/Mapping/src /home/alumno/Documentos/g8-M1/Robotica-2021-2022/Practica-5/Mapping/cmake-build-debug /home/alumno/Documentos/g8-M1/Robotica-2021-2022/Practica-5/Mapping/cmake-build-debug/src /home/alumno/Documentos/g8-M1/Robotica-2021-2022/Practica-5/Mapping/cmake-build-debug/src/CMakeFiles/gotoxy_autogen.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/gotoxy_autogen.dir/depend

