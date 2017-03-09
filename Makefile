include $(shell rospack find mk)/cmake.mk


# Clang is a good compiler to use during development due to its faster compile
# times and more readable output.
# C_compiler=/usr/bin/clang
# CXX_compiler=/usr/bin/clang++

# GCC is better for release mode due to the speed of its output, and its support
# for OpenMP.
C_compiler=/usr/bin/gcc
CXX_compiler=/usr/bin/g++

#acceptable buildTypes: Release/Debug/Profile
buildType=Release
# buildType=Debug

ifeq ($(buildType),Debug)
	build_dir=build_debug
else
	build_dir=build
endif

.SILENT:

all: build/CMakeLists.txt.build
	$(MAKE) --no-print-directory -C $(build_dir)

$(build_dir):
	mkdir $(build_dir)

$(build_dir)/CMakeLists.txt.build: CMakeLists.txt $(build_dir)
	cd $(build_dir) && cmake -DCMAKE_BUILD_TYPE=$(buildType) \
		-DCMAKE_CXX_COMPILER=$(CXX_compiler) \
		-DCMAKE_C_COMPILER=$(C_compiler) \
		-Wno-dev .. && \
		cp ../CMakeLists.txt CMakeLists.txt.build

clean:
	$(MAKE) --no-print-directory -C $(build_dir) clean

cleanup_cache:
	rm -rf $(build_dir)
	rm -rf src/enml
	rm -rf src/third_party/ceres-solver/build
