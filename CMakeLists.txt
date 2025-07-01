cmake_minimum_required(VERSION 3.0.2)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules/")
project(dgcnn)


##############
## Compiler ##
##############

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")

# Check C++20 or C++17 support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++20" COMPILER_SUPPORTS_CXX20)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
if(COMPILER_SUPPORTS_CXX20)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20")
    add_definitions(-DCOMPILEDWITHC20)
    message(" Using flag -std=c++20.")
elseif(COMPILER_SUPPORTS_CXX17)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
    add_definitions(-DCOMPILEDWITHC17)
    message(" Using flag -std=c++17.")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++20 or C++17 support. MULTI_ROBOT_SLAMON needs at least C++17 support. Please use a different C++ compiler.")
endif()


#######################
## Required packages ##
#######################

find_package(
    catkin REQUIRED COMPONENTS
        tf
        roscpp
        std_msgs
        nav_msgs
        cv_bridge
        sensor_msgs
        geometry_msgs
        image_geometry
        image_transport
        message_filters
        message_generation
)


###################################
## Catkin specific configuration ##
###################################

# Declare custom ROS messages
add_message_files(
    DIRECTORY 
        msg 
    FILES
        Chart.msg
        Link.msg
        Node.msg
)

# Declare custom ROS services


# Generate custom ROS messages & services                    
generate_messages(
    DEPENDENCIES
        std_msgs
        sensor_msgs
        geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

###########
## Build ##
###########

include_directories(include 
                    ${catkin_INCLUDE_DIRS} 
)

