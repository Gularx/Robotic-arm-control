cmake_minimum_required(VERSION 3.17)
project(API_QT)

set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
set(CMAKE_CXX_STANDARD 17)

#set(CMAKE_PREFIX_PATH "D:/software/Qt/6.7.1/mingw_64")
#set(CMAKE_AUTOMOC ON)
#
#set(QT_VERSION 6)
#
##链接必要库文件
#set(REQUIRED_LIBS Core Gui Widgets Multimedia)
#set(REQUIRED_LIBS_QUALIFIED Qt6::Core Qt6::Gui Qt6::Widgets Qt6::Multimedia)
#
##查找对应功能包
#find_package(Qt${QT_VERSION} COMPONENTS ${REQUIRED_LIBS} REQUIRED)

# 链接库
link_directories(lib)


# 链接头文件
include_directories(include/robot
        include/haptic
        Application)

file(GLOB_RECURSE SOURCES "Application/*.*")

add_executable(API_QT main.cpp ${SOURCES})

target_link_libraries(API_QT ${REQUIRED_LIBS_QUALIFIED})

target_link_libraries(API_QT
        dhdms64.lib
        drdms64.lib
        RM_Serviced.lib
)