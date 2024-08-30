cmake_minimum_required(VERSION 3.26)
project(rm)

set(CMAKE_CXX_STANDARD 17)

link_directories(lib)
include_directories(include)

add_executable(rm main.cpp)

target_link_libraries(rm
        dhdms64.lib
        drdms64.lib
        RM_Serviced.lib
)

