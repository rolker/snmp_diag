cmake_minimum_required(VERSION 2.8.3)
project(snmp_diag)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    diagnostic_msgs
)


catkin_package(
    CATKIN_DEPENDS
        roscpp
        diagnostic_msgs
    #INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)

include_directories(include ${catkin_INCLUDE_DIRS})

find_library(NETSNMP snmp)

add_executable(${PROJECT_NAME}_node src/snmp_diag.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${NETSNMP})

install(TARGETS ${PROJECT_NAME}_node
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

