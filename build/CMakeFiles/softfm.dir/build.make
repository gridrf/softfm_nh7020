# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/billue/source/softfm_nh7020

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/billue/source/softfm_nh7020/build

# Include any dependencies generated for this target.
include CMakeFiles/softfm.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/softfm.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/softfm.dir/flags.make

CMakeFiles/softfm.dir/main.cc.o: CMakeFiles/softfm.dir/flags.make
CMakeFiles/softfm.dir/main.cc.o: ../main.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/billue/source/softfm_nh7020/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/softfm.dir/main.cc.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/softfm.dir/main.cc.o -c /home/billue/source/softfm_nh7020/main.cc

CMakeFiles/softfm.dir/main.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/softfm.dir/main.cc.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/billue/source/softfm_nh7020/main.cc > CMakeFiles/softfm.dir/main.cc.i

CMakeFiles/softfm.dir/main.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/softfm.dir/main.cc.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/billue/source/softfm_nh7020/main.cc -o CMakeFiles/softfm.dir/main.cc.s

CMakeFiles/softfm.dir/main.cc.o.requires:

.PHONY : CMakeFiles/softfm.dir/main.cc.o.requires

CMakeFiles/softfm.dir/main.cc.o.provides: CMakeFiles/softfm.dir/main.cc.o.requires
	$(MAKE) -f CMakeFiles/softfm.dir/build.make CMakeFiles/softfm.dir/main.cc.o.provides.build
.PHONY : CMakeFiles/softfm.dir/main.cc.o.provides

CMakeFiles/softfm.dir/main.cc.o.provides.build: CMakeFiles/softfm.dir/main.cc.o


CMakeFiles/softfm.dir/RtlSdrSource.cc.o: CMakeFiles/softfm.dir/flags.make
CMakeFiles/softfm.dir/RtlSdrSource.cc.o: ../RtlSdrSource.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/billue/source/softfm_nh7020/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/softfm.dir/RtlSdrSource.cc.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/softfm.dir/RtlSdrSource.cc.o -c /home/billue/source/softfm_nh7020/RtlSdrSource.cc

CMakeFiles/softfm.dir/RtlSdrSource.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/softfm.dir/RtlSdrSource.cc.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/billue/source/softfm_nh7020/RtlSdrSource.cc > CMakeFiles/softfm.dir/RtlSdrSource.cc.i

CMakeFiles/softfm.dir/RtlSdrSource.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/softfm.dir/RtlSdrSource.cc.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/billue/source/softfm_nh7020/RtlSdrSource.cc -o CMakeFiles/softfm.dir/RtlSdrSource.cc.s

CMakeFiles/softfm.dir/RtlSdrSource.cc.o.requires:

.PHONY : CMakeFiles/softfm.dir/RtlSdrSource.cc.o.requires

CMakeFiles/softfm.dir/RtlSdrSource.cc.o.provides: CMakeFiles/softfm.dir/RtlSdrSource.cc.o.requires
	$(MAKE) -f CMakeFiles/softfm.dir/build.make CMakeFiles/softfm.dir/RtlSdrSource.cc.o.provides.build
.PHONY : CMakeFiles/softfm.dir/RtlSdrSource.cc.o.provides

CMakeFiles/softfm.dir/RtlSdrSource.cc.o.provides.build: CMakeFiles/softfm.dir/RtlSdrSource.cc.o


CMakeFiles/softfm.dir/Filter.cc.o: CMakeFiles/softfm.dir/flags.make
CMakeFiles/softfm.dir/Filter.cc.o: ../Filter.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/billue/source/softfm_nh7020/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/softfm.dir/Filter.cc.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/softfm.dir/Filter.cc.o -c /home/billue/source/softfm_nh7020/Filter.cc

CMakeFiles/softfm.dir/Filter.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/softfm.dir/Filter.cc.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/billue/source/softfm_nh7020/Filter.cc > CMakeFiles/softfm.dir/Filter.cc.i

CMakeFiles/softfm.dir/Filter.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/softfm.dir/Filter.cc.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/billue/source/softfm_nh7020/Filter.cc -o CMakeFiles/softfm.dir/Filter.cc.s

CMakeFiles/softfm.dir/Filter.cc.o.requires:

.PHONY : CMakeFiles/softfm.dir/Filter.cc.o.requires

CMakeFiles/softfm.dir/Filter.cc.o.provides: CMakeFiles/softfm.dir/Filter.cc.o.requires
	$(MAKE) -f CMakeFiles/softfm.dir/build.make CMakeFiles/softfm.dir/Filter.cc.o.provides.build
.PHONY : CMakeFiles/softfm.dir/Filter.cc.o.provides

CMakeFiles/softfm.dir/Filter.cc.o.provides.build: CMakeFiles/softfm.dir/Filter.cc.o


CMakeFiles/softfm.dir/FmDecode.cc.o: CMakeFiles/softfm.dir/flags.make
CMakeFiles/softfm.dir/FmDecode.cc.o: ../FmDecode.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/billue/source/softfm_nh7020/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/softfm.dir/FmDecode.cc.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/softfm.dir/FmDecode.cc.o -c /home/billue/source/softfm_nh7020/FmDecode.cc

CMakeFiles/softfm.dir/FmDecode.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/softfm.dir/FmDecode.cc.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/billue/source/softfm_nh7020/FmDecode.cc > CMakeFiles/softfm.dir/FmDecode.cc.i

CMakeFiles/softfm.dir/FmDecode.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/softfm.dir/FmDecode.cc.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/billue/source/softfm_nh7020/FmDecode.cc -o CMakeFiles/softfm.dir/FmDecode.cc.s

CMakeFiles/softfm.dir/FmDecode.cc.o.requires:

.PHONY : CMakeFiles/softfm.dir/FmDecode.cc.o.requires

CMakeFiles/softfm.dir/FmDecode.cc.o.provides: CMakeFiles/softfm.dir/FmDecode.cc.o.requires
	$(MAKE) -f CMakeFiles/softfm.dir/build.make CMakeFiles/softfm.dir/FmDecode.cc.o.provides.build
.PHONY : CMakeFiles/softfm.dir/FmDecode.cc.o.provides

CMakeFiles/softfm.dir/FmDecode.cc.o.provides.build: CMakeFiles/softfm.dir/FmDecode.cc.o


CMakeFiles/softfm.dir/AudioOutput.cc.o: CMakeFiles/softfm.dir/flags.make
CMakeFiles/softfm.dir/AudioOutput.cc.o: ../AudioOutput.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/billue/source/softfm_nh7020/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/softfm.dir/AudioOutput.cc.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/softfm.dir/AudioOutput.cc.o -c /home/billue/source/softfm_nh7020/AudioOutput.cc

CMakeFiles/softfm.dir/AudioOutput.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/softfm.dir/AudioOutput.cc.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/billue/source/softfm_nh7020/AudioOutput.cc > CMakeFiles/softfm.dir/AudioOutput.cc.i

CMakeFiles/softfm.dir/AudioOutput.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/softfm.dir/AudioOutput.cc.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/billue/source/softfm_nh7020/AudioOutput.cc -o CMakeFiles/softfm.dir/AudioOutput.cc.s

CMakeFiles/softfm.dir/AudioOutput.cc.o.requires:

.PHONY : CMakeFiles/softfm.dir/AudioOutput.cc.o.requires

CMakeFiles/softfm.dir/AudioOutput.cc.o.provides: CMakeFiles/softfm.dir/AudioOutput.cc.o.requires
	$(MAKE) -f CMakeFiles/softfm.dir/build.make CMakeFiles/softfm.dir/AudioOutput.cc.o.provides.build
.PHONY : CMakeFiles/softfm.dir/AudioOutput.cc.o.provides

CMakeFiles/softfm.dir/AudioOutput.cc.o.provides.build: CMakeFiles/softfm.dir/AudioOutput.cc.o


# Object files for target softfm
softfm_OBJECTS = \
"CMakeFiles/softfm.dir/main.cc.o" \
"CMakeFiles/softfm.dir/RtlSdrSource.cc.o" \
"CMakeFiles/softfm.dir/Filter.cc.o" \
"CMakeFiles/softfm.dir/FmDecode.cc.o" \
"CMakeFiles/softfm.dir/AudioOutput.cc.o"

# External object files for target softfm
softfm_EXTERNAL_OBJECTS =

softfm: CMakeFiles/softfm.dir/main.cc.o
softfm: CMakeFiles/softfm.dir/RtlSdrSource.cc.o
softfm: CMakeFiles/softfm.dir/Filter.cc.o
softfm: CMakeFiles/softfm.dir/FmDecode.cc.o
softfm: CMakeFiles/softfm.dir/AudioOutput.cc.o
softfm: CMakeFiles/softfm.dir/build.make
softfm: /usr/lib/x86_64-linux-gnu/libiio.so
softfm: /usr/local/lib/libad9361.so
softfm: /usr/lib/x86_64-linux-gnu/libasound.so
softfm: CMakeFiles/softfm.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/billue/source/softfm_nh7020/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking CXX executable softfm"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/softfm.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/softfm.dir/build: softfm

.PHONY : CMakeFiles/softfm.dir/build

CMakeFiles/softfm.dir/requires: CMakeFiles/softfm.dir/main.cc.o.requires
CMakeFiles/softfm.dir/requires: CMakeFiles/softfm.dir/RtlSdrSource.cc.o.requires
CMakeFiles/softfm.dir/requires: CMakeFiles/softfm.dir/Filter.cc.o.requires
CMakeFiles/softfm.dir/requires: CMakeFiles/softfm.dir/FmDecode.cc.o.requires
CMakeFiles/softfm.dir/requires: CMakeFiles/softfm.dir/AudioOutput.cc.o.requires

.PHONY : CMakeFiles/softfm.dir/requires

CMakeFiles/softfm.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/softfm.dir/cmake_clean.cmake
.PHONY : CMakeFiles/softfm.dir/clean

CMakeFiles/softfm.dir/depend:
	cd /home/billue/source/softfm_nh7020/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/billue/source/softfm_nh7020 /home/billue/source/softfm_nh7020 /home/billue/source/softfm_nh7020/build /home/billue/source/softfm_nh7020/build /home/billue/source/softfm_nh7020/build/CMakeFiles/softfm.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/softfm.dir/depend

