cmake_minimum_required(VERSION 3.0.2)

project(vio_stereo)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    tf
    cv_bridge
    camera_models
    image_transport)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(OpenCV REQUIRED)

find_package(Ceres REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)

file(GLOB_RECURSE sources "src/*.cpp")

add_library(${PROJECT_NAME} ${sources})

target_include_directories(${PROJECT_NAME} PUBLIC include/${PROJECT_NAME} ${catkin_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES} Eigen3::Eigen)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

add_executable(vio_node src/vio_node.cpp)
target_link_libraries(vio_node ${PROJECT_NAME})

