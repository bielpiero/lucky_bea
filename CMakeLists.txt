cmake_minimum_required(VERSION 2.8.3)
project(lucky_bea)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
find_package(PkgConfig REQUIRED)
find_package(OpenCV REQUIRED)
pkg_search_module(LIBUSB_1 REQUIRED libusb-1.0)
pkg_search_module(SSL_L REQUIRED libssl)
include_directories(include ${LIBUSB_1_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories( /usr/local/Aria/include)
link_directories(/usr/bin)
link_directories(/usr/local/Aria/lib)
link_directories(lib/)

## Declare a cpp library
# add_library(lucky_bea_socketnode src/SocketNode2.cpp)

## Declare a cpp executable
file(GLOB_RECURSE lucky_bea_SOURCES src/*.cpp)
file(GLOB_RECURSE lucky_bea_HEADERS include/*.h)

set(lucky_bea_INCLUDE_DIRS "")
foreach(_headerFile ${lucky_bea_HEADERS})
	#MESSAGE("Now Globbing.... ${lucky_bea_HEADERS}")
	get_filename_component(_dir ${_headerFile} PATH)
	list(APPEND lucky_bea_INCLUDE_DIRS ${_dir})
endforeach()
list(REMOVE_DUPLICATES lucky_bea_INCLUDE_DIRS)

include_directories(${lucky_bea_INCLUDE_DIRS})
add_executable(lucky_bea 
	${lucky_bea_SOURCES}
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(lucky_bea lucky_bea_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(lucky_bea
  #${catkin_LIBRARIES}
  ${LIBUSB_1_LIBRARIES}
  ${OpenCV_LIBRARIES}
  Aria pthread rt
  ftd2xx
  xml2
  ltkcpp
  ltkcppimpinj
  dl ssl crypto
)
set_target_properties(lucky_bea PROPERTIES COMPILE_FLAGS "-m32")
set_target_properties(lucky_bea PROPERTIES COMPILE_FLAGS "-Wno-write-strings")
set_target_properties(lucky_bea PROPERTIES COMPILE_FLAGS "-fPIC")
#set_target_properties(lucky_bea PROPERTIES LINK_FLAGS "-Wl")

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
#install(TARGETS lucky_bea
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Mark cpp header files for installation
#install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
#)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   "src/conf/BeaConSuerte.xml"
#   DESTINATION lib/${PROJECT_NAME}/conf/
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_lucky_bea.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
