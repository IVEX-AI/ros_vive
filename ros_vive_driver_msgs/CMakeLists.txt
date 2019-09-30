cmake_minimum_required(VERSION 2.8.3)

project(ros_vive_driver_msgs)

find_package(catkin REQUIRED COMPONENTS
    message_generation
)


add_message_files(
    FILES
    ControllerState.msg
)
add_service_files(
    FILES
    VibrateControllerContinuous.srv
    VibrateControllerPulse.srv
)
generate_messages()


catkin_package(
    DEPENDS
    CATKIN_DEPENDS message_runtime
    INCLUDE_DIRS
    LIBRARIES
)