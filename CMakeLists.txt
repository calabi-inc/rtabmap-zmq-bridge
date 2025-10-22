cmake_minimum_required(VERSION 3.16)
project(rtabmap_zmq_bridge CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)

include_directories(${OpenCV_INCLUDE_DIRS} ${ZMQ_INCLUDE_DIRS} /usr/local/include)
link_directories(/usr/local/lib ${ZMQ_LIBRARY_DIRS})

add_executable(${PROJECT_NAME}
	src/camera_zmq.cpp
	src/bridge.cpp
)

target_link_libraries(${PROJECT_NAME}
	${OpenCV_LIBS}
	zmq
	rtabmap_core rtabmap_utilite rtabmap
	pthread
)
