cmake_minimum_required(VERSION 2.8)

project(tjsp_attack_2020)

set(CMAKE_CXX_STANDARD 14)  # for chrono
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-Ofast -march=native -pthread")

set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-std=c++14)

#====================== OpenCV Config ============================
#set(OpenCV_DIR /home/youngxi/opt/OpenCV/opencv-3.4.9/build)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
find_package (Threads REQUIRED)

message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
#=================================================================

# Find Boost
find_package(Boost REQUIRED COMPONENTS thread)
if (!Boost_FOUND)
    message("-- Found Boost NOT FOUND")
endif ()

find_package(catkin REQUIRED COMPONENTS
  costmap_2d  
  sensor_msgs
  roscpp
  cv_bridge
  rospy
  std_msgs
  tf
  roborts_msgs
  message_generation
)

add_service_files(FILES img.srv)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    cv_bridge
    geometry_msgs
    image_transport
    roscpp
    sensor_msgs
    std_msgs
    tf
    rospy
    message_runtime
  DEPENDS
    OpenCV

)

# set(LIB_THREAD_DIR /lib/x86_64-linux-gnu)
# add_library(libpthread SHARED IMPORTED)
# set_target_properties(libopencv_imgproc PROPERTIES IMPORTED_LOCATION ${LIB_THREAD_DIR}/libpthread.so.0)

#/opt/DahuaTech/MVviewer/include
include_directories(source include)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
add_executable(armorAttack armor_attack.cpp)
TARGET_LINK_LIBRARIES(armorAttack ${catkin_LIBRARIES})
target_link_libraries(armorAttack ${OpenCV_LIBS})
# target_link_libraries(test /home/youngxi/opt/mindvision/lib/x64/libMVSDK.so)
#file(GLOB HikVision "/opt/MVS/Samples_LinuxSDK/lib/64/*.so")
#target_link_libraries(attack ${HikVision})
target_link_libraries(armorAttack ${Boost_LIBRARIES} usb-1.0)
add_dependencies(armorAttack ${PROJECT_NAME}_gencpp)
# target_link_libraries(test libpthread)

# FIND_PACKAGE(OpenMP REQUIRED)
# if (OPENMP_FOUND)
#     message("OPENMP FOUND")
#     set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
#     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
#     set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
# endif ()

