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
CMAKE_COMMAND = /opt/clion-2021.2.1/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /opt/clion-2021.2.1/bin/cmake/linux/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/josel/Documentos/Curso_21_22/Robotica/Entrega-1/ContadorStd

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/josel/Documentos/Curso_21_22/Robotica/Entrega-1/ContadorStd/cmake-build-timersimple_robotica

# Include any dependencies generated for this target.
include CMakeFiles/ejemplo1.dir/depend.make
# Include the progress variables for this target.
include CMakeFiles/ejemplo1.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/ejemplo1.dir/flags.make

moc_ejemplo1.cpp: ../ejemplo1.h
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/josel/Documentos/Curso_21_22/Robotica/Entrega-1/ContadorStd/cmake-build-timersimple_robotica/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating moc_ejemplo1.cpp"
	/usr/lib/qt5/bin/moc @/home/josel/Documentos/Curso_21_22/Robotica/Entrega-1/ContadorStd/cmake-build-timersimple_robotica/moc_ejemplo1.cpp_parameters

ui_counterDlg.h: ../counterDlg.ui
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/josel/Documentos/Curso_21_22/Robotica/Entrega-1/ContadorStd/cmake-build-timersimple_robotica/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating ui_counterDlg.h"
	/usr/lib/qt5/bin/uic -o /home/josel/Documentos/Curso_21_22/Robotica/Entrega-1/ContadorStd/cmake-build-timersimple_robotica/ui_counterDlg.h /home/josel/Documentos/Curso_21_22/Robotica/Entrega-1/ContadorStd/counterDlg.ui

CMakeFiles/ejemplo1.dir/ejemplo1.cpp.o: CMakeFiles/ejemplo1.dir/flags.make
CMakeFiles/ejemplo1.dir/ejemplo1.cpp.o: ../ejemplo1.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/josel/Documentos/Curso_21_22/Robotica/Entrega-1/ContadorStd/cmake-build-timersimple_robotica/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/ejemplo1.dir/ejemplo1.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ejemplo1.dir/ejemplo1.cpp.o -c /home/josel/Documentos/Curso_21_22/Robotica/Entrega-1/ContadorStd/ejemplo1.cpp

CMakeFiles/ejemplo1.dir/ejemplo1.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ejemplo1.dir/ejemplo1.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/josel/Documentos/Curso_21_22/Robotica/Entrega-1/ContadorStd/ejemplo1.cpp > CMakeFiles/ejemplo1.dir/ejemplo1.cpp.i

CMakeFiles/ejemplo1.dir/ejemplo1.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ejemplo1.dir/ejemplo1.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/josel/Documentos/Curso_21_22/Robotica/Entrega-1/ContadorStd/ejemplo1.cpp -o CMakeFiles/ejemplo1.dir/ejemplo1.cpp.s

CMakeFiles/ejemplo1.dir/main.cpp.o: CMakeFiles/ejemplo1.dir/flags.make
CMakeFiles/ejemplo1.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/josel/Documentos/Curso_21_22/Robotica/Entrega-1/ContadorStd/cmake-build-timersimple_robotica/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/ejemplo1.dir/main.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ejemplo1.dir/main.cpp.o -c /home/josel/Documentos/Curso_21_22/Robotica/Entrega-1/ContadorStd/main.cpp

CMakeFiles/ejemplo1.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ejemplo1.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/josel/Documentos/Curso_21_22/Robotica/Entrega-1/ContadorStd/main.cpp > CMakeFiles/ejemplo1.dir/main.cpp.i

CMakeFiles/ejemplo1.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ejemplo1.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/josel/Documentos/Curso_21_22/Robotica/Entrega-1/ContadorStd/main.cpp -o CMakeFiles/ejemplo1.dir/main.cpp.s

CMakeFiles/ejemplo1.dir/moc_ejemplo1.cpp.o: CMakeFiles/ejemplo1.dir/flags.make
CMakeFiles/ejemplo1.dir/moc_ejemplo1.cpp.o: moc_ejemplo1.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/josel/Documentos/Curso_21_22/Robotica/Entrega-1/ContadorStd/cmake-build-timersimple_robotica/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/ejemplo1.dir/moc_ejemplo1.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ejemplo1.dir/moc_ejemplo1.cpp.o -c /home/josel/Documentos/Curso_21_22/Robotica/Entrega-1/ContadorStd/cmake-build-timersimple_robotica/moc_ejemplo1.cpp

CMakeFiles/ejemplo1.dir/moc_ejemplo1.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ejemplo1.dir/moc_ejemplo1.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/josel/Documentos/Curso_21_22/Robotica/Entrega-1/ContadorStd/cmake-build-timersimple_robotica/moc_ejemplo1.cpp > CMakeFiles/ejemplo1.dir/moc_ejemplo1.cpp.i

CMakeFiles/ejemplo1.dir/moc_ejemplo1.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ejemplo1.dir/moc_ejemplo1.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/josel/Documentos/Curso_21_22/Robotica/Entrega-1/ContadorStd/cmake-build-timersimple_robotica/moc_ejemplo1.cpp -o CMakeFiles/ejemplo1.dir/moc_ejemplo1.cpp.s

# Object files for target ejemplo1
ejemplo1_OBJECTS = \
"CMakeFiles/ejemplo1.dir/ejemplo1.cpp.o" \
"CMakeFiles/ejemplo1.dir/main.cpp.o" \
"CMakeFiles/ejemplo1.dir/moc_ejemplo1.cpp.o"

# External object files for target ejemplo1
ejemplo1_EXTERNAL_OBJECTS =

ejemplo1: CMakeFiles/ejemplo1.dir/ejemplo1.cpp.o
ejemplo1: CMakeFiles/ejemplo1.dir/main.cpp.o
ejemplo1: CMakeFiles/ejemplo1.dir/moc_ejemplo1.cpp.o
ejemplo1: CMakeFiles/ejemplo1.dir/build.make
ejemplo1: /usr/lib/x86_64-linux-gnu/libQt5Sql.so.5.12.8
ejemplo1: /usr/lib/x86_64-linux-gnu/libQt5OpenGL.so.5.12.8
ejemplo1: /usr/lib/x86_64-linux-gnu/libQt5Xml.so.5.12.8
ejemplo1: /usr/lib/x86_64-linux-gnu/libQt5XmlPatterns.so.5.12.8
ejemplo1: /usr/lib/x86_64-linux-gnu/libQt5Widgets.so.5.12.8
ejemplo1: /usr/lib/x86_64-linux-gnu/libQt5Gui.so.5.12.8
ejemplo1: /usr/lib/x86_64-linux-gnu/libQt5Network.so.5.12.8
ejemplo1: /usr/lib/x86_64-linux-gnu/libQt5Core.so.5.12.8
ejemplo1: CMakeFiles/ejemplo1.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/josel/Documentos/Curso_21_22/Robotica/Entrega-1/ContadorStd/cmake-build-timersimple_robotica/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking CXX executable ejemplo1"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ejemplo1.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/ejemplo1.dir/build: ejemplo1
.PHONY : CMakeFiles/ejemplo1.dir/build

CMakeFiles/ejemplo1.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ejemplo1.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ejemplo1.dir/clean

CMakeFiles/ejemplo1.dir/depend: moc_ejemplo1.cpp
CMakeFiles/ejemplo1.dir/depend: ui_counterDlg.h
	cd /home/josel/Documentos/Curso_21_22/Robotica/Entrega-1/ContadorStd/cmake-build-timersimple_robotica && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/josel/Documentos/Curso_21_22/Robotica/Entrega-1/ContadorStd /home/josel/Documentos/Curso_21_22/Robotica/Entrega-1/ContadorStd /home/josel/Documentos/Curso_21_22/Robotica/Entrega-1/ContadorStd/cmake-build-timersimple_robotica /home/josel/Documentos/Curso_21_22/Robotica/Entrega-1/ContadorStd/cmake-build-timersimple_robotica /home/josel/Documentos/Curso_21_22/Robotica/Entrega-1/ContadorStd/cmake-build-timersimple_robotica/CMakeFiles/ejemplo1.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ejemplo1.dir/depend

