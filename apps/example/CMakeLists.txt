project(demo)
cmake_minimum_required(VERSION 3.0)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/../../build/")
include(${CMAKE_MODULE_PATH}inno_lidar_base.cmake)

add_executable(demo ${PROJECT_SOURCE_DIR}/demo.cpp)

add_executable(sphere2xyz ${PROJECT_SOURCE_DIR}/sphere2xyz.cpp)


if(CMAKE_SYSTEM_NAME MATCHES "Windows")
    target_link_libraries(demo PRIVATE innolidarsdkclient innolidarsdkcommon innolidarutils ws2_32 wsock32)
    target_link_libraries(sphere2xyz PRIVATE innolidarsdkclient innolidarsdkcommon innolidarutils ws2_32 wsock32)
elseif(IS_QNX)
    set(CMAKE_FIND_LIBRARY_SUFFIXES .a)
    target_link_libraries(demo PRIVATE libinnolidarsdkclient.a libinnolidarsdkcommon.a libinnolidarutils.a socket)
    target_link_libraries(sphere2xyz PRIVATE libinnolidarsdkclient.a libinnolidarsdkcommon.a libinnolidarutils.a socket)
else()
    set(CMAKE_FIND_LIBRARY_SUFFIXES .a)
    target_link_libraries(demo PRIVATE libinnolidarsdkclient.a libinnolidarsdkcommon.a libinnolidarutils.a pthread)
    target_link_libraries(sphere2xyz PRIVATE libinnolidarsdkclient.a libinnolidarsdkcommon.a libinnolidarutils.a pthread)
endif()

########################
set(targets demo sphere2xyz)

foreach(target ${targets})
    INSTALL(TARGETS ${target}
        RUNTIME DESTINATION ${PROJECT_SOURCE_DIR}/
    )
endforeach()
