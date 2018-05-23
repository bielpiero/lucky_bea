cmake_minimum_required(VERSION 2.8.3)
project(lucky_bea)

###########
## Build ##
###########

## Specify additional locations of header files

set(CMAKE_CXX_STANDARD 11) # C++11...
set(CMAKE_CXX_STANDARD_REQUIRED ON) #...is required...
set(CMAKE_CXX_EXTENSIONS OFF) #...without compiler extensions like gnu++11

## Your package locations should be listed before other locations
find_package(PkgConfig REQUIRED)
find_package(OpenCV REQUIRED)
pkg_search_module(LIBUSB_1 REQUIRED libusb-1.0)
pkg_search_module(SSL_L REQUIRED libssl)
include_directories(include ${LIBUSB_1_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories( /usr/local/Aria/include)
link_directories(/usr/local/Aria/lib)
link_directories(lib/)

add_definitions(-Dlinux)
#add_definitions(-DFL_CPP98)
#add_definitions(-std=c++98)

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
# add_dependencies(lucky_bea lucky_bea_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(lucky_bea
  #${catkin_LIBRARIES}
  ${LIBUSB_1_LIBRARIES}
  ${OpenCV_LIBRARIES}
  Aria pthread rt
  ftd2xx
  xml2
  dxl_x86_cpp
  fuzzylite
  ltkcpp
  ltkcppimpinj
  curl
  dl ssl crypto
)

set_target_properties(lucky_bea PROPERTIES COMPILE_FLAGS "-Wall")
set_target_properties(lucky_bea PROPERTIES COMPILE_FLAGS "-m32")
set_target_properties(lucky_bea PROPERTIES COMPILE_FLAGS "-Wno-write-strings")
set_target_properties(lucky_bea PROPERTIES COMPILE_FLAGS "-fPIC")
#set_target_properties(lucky_bea PROPERTIES LINK_FLAGS "-Wl")


