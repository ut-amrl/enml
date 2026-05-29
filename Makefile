#acceptable build_types: Release/Debug/Profile
build_type=Release
# build_type=Debug

AMRL_MSGS_PREFIX ?= ../amrl_msgs/install

ifneq ($(wildcard $(AMRL_MSGS_PREFIX)/share/amrl_msgs/cmake/amrl_msgsConfig.cmake),)
export AMENT_PREFIX_PATH := $(abspath $(AMRL_MSGS_PREFIX)):$(AMENT_PREFIX_PATH)
export CMAKE_PREFIX_PATH := $(abspath $(AMRL_MSGS_PREFIX)):$(CMAKE_PREFIX_PATH)
export LD_LIBRARY_PATH := $(abspath $(AMRL_MSGS_PREFIX))/lib:$(LD_LIBRARY_PATH)
endif

.SILENT:

all: build build/CMakeLists.txt.copy
	$(info Build_type is [${build_type}])
	$(MAKE) --no-print-directory -C build

clean:
	rm -rf build bin lib msg_gen src/enml install

build/CMakeLists.txt.copy: build CMakeLists.txt Makefile
	cd build && cmake -DCMAKE_BUILD_TYPE=$(build_type) \
	  -DCMAKE_INSTALL_PREFIX=../install \
	  -DPython3_EXECUTABLE=/usr/bin/python3 ..
	cp CMakeLists.txt build/CMakeLists.txt.copy

build:
	mkdir -p build
