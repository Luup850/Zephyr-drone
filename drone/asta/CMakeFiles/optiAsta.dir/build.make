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
CMAKE_SOURCE_DIR = /home/local/mdch/asta

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/local/mdch/asta

# Include any dependencies generated for this target.
include CMakeFiles/optiAsta.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/optiAsta.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/optiAsta.dir/flags.make

CMakeFiles/optiAsta.dir/main.cpp.o: CMakeFiles/optiAsta.dir/flags.make
CMakeFiles/optiAsta.dir/main.cpp.o: main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/local/mdch/asta/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/optiAsta.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/optiAsta.dir/main.cpp.o -c /home/local/mdch/asta/main.cpp

CMakeFiles/optiAsta.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/optiAsta.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/local/mdch/asta/main.cpp > CMakeFiles/optiAsta.dir/main.cpp.i

CMakeFiles/optiAsta.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/optiAsta.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/local/mdch/asta/main.cpp -o CMakeFiles/optiAsta.dir/main.cpp.s

CMakeFiles/optiAsta.dir/optidata.cpp.o: CMakeFiles/optiAsta.dir/flags.make
CMakeFiles/optiAsta.dir/optidata.cpp.o: optidata.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/local/mdch/asta/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/optiAsta.dir/optidata.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/optiAsta.dir/optidata.cpp.o -c /home/local/mdch/asta/optidata.cpp

CMakeFiles/optiAsta.dir/optidata.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/optiAsta.dir/optidata.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/local/mdch/asta/optidata.cpp > CMakeFiles/optiAsta.dir/optidata.cpp.i

CMakeFiles/optiAsta.dir/optidata.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/optiAsta.dir/optidata.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/local/mdch/asta/optidata.cpp -o CMakeFiles/optiAsta.dir/optidata.cpp.s

CMakeFiles/optiAsta.dir/PacketClient.cpp.o: CMakeFiles/optiAsta.dir/flags.make
CMakeFiles/optiAsta.dir/PacketClient.cpp.o: PacketClient.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/local/mdch/asta/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/optiAsta.dir/PacketClient.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/optiAsta.dir/PacketClient.cpp.o -c /home/local/mdch/asta/PacketClient.cpp

CMakeFiles/optiAsta.dir/PacketClient.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/optiAsta.dir/PacketClient.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/local/mdch/asta/PacketClient.cpp > CMakeFiles/optiAsta.dir/PacketClient.cpp.i

CMakeFiles/optiAsta.dir/PacketClient.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/optiAsta.dir/PacketClient.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/local/mdch/asta/PacketClient.cpp -o CMakeFiles/optiAsta.dir/PacketClient.cpp.s

# Object files for target optiAsta
optiAsta_OBJECTS = \
"CMakeFiles/optiAsta.dir/main.cpp.o" \
"CMakeFiles/optiAsta.dir/optidata.cpp.o" \
"CMakeFiles/optiAsta.dir/PacketClient.cpp.o"

# External object files for target optiAsta
optiAsta_EXTERNAL_OBJECTS =

optiAsta: CMakeFiles/optiAsta.dir/main.cpp.o
optiAsta: CMakeFiles/optiAsta.dir/optidata.cpp.o
optiAsta: CMakeFiles/optiAsta.dir/PacketClient.cpp.o
optiAsta: CMakeFiles/optiAsta.dir/build.make
optiAsta: CMakeFiles/optiAsta.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/local/mdch/asta/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable optiAsta"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/optiAsta.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/optiAsta.dir/build: optiAsta

.PHONY : CMakeFiles/optiAsta.dir/build

CMakeFiles/optiAsta.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/optiAsta.dir/cmake_clean.cmake
.PHONY : CMakeFiles/optiAsta.dir/clean

CMakeFiles/optiAsta.dir/depend:
	cd /home/local/mdch/asta && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/local/mdch/asta /home/local/mdch/asta /home/local/mdch/asta /home/local/mdch/asta /home/local/mdch/asta/CMakeFiles/optiAsta.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/optiAsta.dir/depend
