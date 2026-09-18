cmake_minimum_required(VERSION 3.14)
project(caen_v1730)

set(CMAKE_CXX_STANDARD 17)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(pybind11 REQUIRED)

include_directories(caen_v1730 PUBLIC
    /home/midptf/packages/midas/include/ /home/midptf/environments/phidget/include/ /usr/include/python3.10/ /home/midptf/packages/midas/drivers/vme/vmic/ /home/midptf/packages/root/include /usr/include
    )
include_directories(caen_v1730 PRIVATE ${CMAKE_SOURCE_DIR}/include)


pybind11_add_module(caen_v1730
    src/bindings.cpp
    src/digitizer.cpp
)

target_link_libraries(caen_v1730 PRIVATE
    libCAENComm.so libCAENVME.so libCAENDigitizer.so
)
