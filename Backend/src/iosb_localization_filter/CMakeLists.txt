cmake_minimum_required(VERSION 3.0.2)
project(iosb_localization_filter)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

add_message_files(FILES FilterState.msg)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS message_runtime)
