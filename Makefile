# include $(shell rospack find mk)/cmake.mk

#acceptable build_types: Release/Debug/Profile
build_type=Release
# build_type=Debug

.SILENT:

all: build build/CMakeLists.txt.copy amrl_msgs_make graph_navigation/bin/navigation 
	$(info Build_type is [${build_type}])
	$(info Build_mode is [${build_mode}])
	$(MAKE) --no-print-directory -C build


clean:
	rm -rf build bin lib
	cd graph_navigation && rm -rf build bin lib
	cd amrl_msgs && rm -rf build msg_gen srv_gen

build/CMakeLists.txt.copy: build CMakeLists.txt Makefile
	cd build && cmake -DCMAKE_BUILD_TYPE=$(build_type) ..
	cp CMakeLists.txt build/CMakeLists.txt.copy

build:
	mkdir -p build

graph_navigation/bin/navigation:
	cd graph_navigation && $(MAKE) -e CMAKE_CUDA_ARCHITECTURES=86 

amrl_msgs_make:
	cd amrl_msgs && $(MAKE)