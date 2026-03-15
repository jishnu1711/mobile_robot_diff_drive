cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS
    components/micro_ros_espidf
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(amr_kinco_diff_drive)
