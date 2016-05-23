cmake_minimum_required (VERSION 2.8)
project (pixy_node CXX C)

set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake" )
set (LIBPOSITIONING_LIBRARY "${CMAKE_SOURCE_DIR}/lib/indoor_positioning")

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    roscpp
    )
find_package ( libpixyusb REQUIRED )
find_package ( libusb-1.0 REQUIRED )
find_package ( Boost 1.49 COMPONENTS thread system chrono REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
    CATKIN_DEPENDS roscpp geometry_msgs
    DEPENDS Boost
)

###########
## Build ##
###########
file(STRINGS "cmake/VERSION" LIBPIXY_VERSION)
add_definitions(-D__LIBPIXY_VERSION__="${LIBPIXY_VERSION}")

include_directories (${LIBPOSITIONING_LIBRARY}
    src
    include
    ../../common
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    ${LIBUSB_1_INCLUDE_DIRS})

add_library (pos ${LIBPOSITIONING_LIBRARY}/drone_loc.c)

# Add executable here... #
add_executable (pixy_node main.cpp)

# libpixyusb should always come before libboost and libusb #
target_link_libraries (pixy_node pixyusb pos ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${LIBUSB_1_LIBRARY})
#target_link_libraries (Pixy_Node ${Boost_LIBRARIES})
#target_link_libraries (Pixy_Node ${LIBUSB_1_LIBRARY})

#############
## Install ##
#############
install(TARGETS pixy_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )








