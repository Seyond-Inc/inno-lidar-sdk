project(get_pcd)
cmake_minimum_required( VERSION 3.0 )

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/../../../build/")
include(${CMAKE_MODULE_PATH}inno_lidar_base.cmake)

set(SOURCES_GET_PCD get_pcd.cpp)

add_executable(${PROJECT_NAME} ${SOURCES_GET_PCD})

if(CMAKE_SYSTEM_NAME MATCHES "Windows")
    # include MSVC and MingW
    target_link_libraries(${PROJECT_NAME} PRIVATE innolidarsdkclient innolidarsdkcommon innolidarutils ws2_32 wsock32)
elseif(IS_QNX)
    set(CMAKE_FIND_LIBRARY_SUFFIXES .a)
    target_link_libraries(${PROJECT_NAME} PRIVATE innolidarsdkclient.a innolidarsdkcommon.a innolidarutils.a socket)
else()
    set(CMAKE_FIND_LIBRARY_SUFFIXES .a)
    target_link_libraries(${PROJECT_NAME} PRIVATE innolidarsdkclient.a innolidarsdkcommon.a innolidarutils.a pthread)
endif()

########################
INSTALL(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION ${PROJECT_SOURCE_DIR}/
)
