cmake_minimum_required(VERSION 3.16)
project(rtabmap_zmq_bridge CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(RTABMap REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS} ${ZMQ_INCLUDE_DIRS} /usr/local/include)
include_directories(${EIGEN3_INCLUDE_DIRS})
# Fallback for Eigen include path if variables are empty
include_directories(/usr/include/eigen3)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
include_directories(${RTABMap_INCLUDE_DIRS})
link_directories(/usr/local/lib ${ZMQ_LIBRARY_DIRS})

# Ensure RTAB-Map headers are found (installed under /usr/local/include/rtabmap*)
file(GLOB RTABMAP_INC_HINTS "/usr/local/include/rtabmap*")
if(RTABMAP_INC_HINTS)
	include_directories(${RTABMAP_INC_HINTS})
endif()

add_executable(${PROJECT_NAME}
	src/camera_zmq.cpp
	src/bridge.cpp
)

target_link_libraries(${PROJECT_NAME}
	${OpenCV_LIBS}
	zmq
	${RTABMap_LIBRARIES}
	${PCL_LIBRARIES}
	Eigen3::Eigen
	pthread
)
