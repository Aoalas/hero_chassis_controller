cmake_minimum_required(VERSION 3.10)
project(hero_chassis_controller)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-Wall -Werror)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        roslint
        controller_interface
        hardware_interface
        forward_command_controller
        pluginlib
        dynamic_reconfigure
        control_toolbox
        tf
)

generate_dynamic_reconfigure_options(
        cfg/PidConfig.cfg
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS
        roscpp
        roslint
        controller_interface
        hardware_interface
        forward_command_controller
        pluginlib
        dynamic_reconfigure
        control_toolbox
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME}
        src/hero_chassis_controller.cpp
)

add_dependencies(${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
)

roslint_cpp()
