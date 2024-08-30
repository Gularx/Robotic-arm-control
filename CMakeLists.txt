cmake_minimum_required(VERSION 3.26)
project(RM_QT_API)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_PREFIX_PATH "D:/software/Qt/6.7.1/msvc2019_64")
set(CMAKE_AUTOMOC ON)
set(QT_VERSION 6)
#链接必要库文件
set(REQUIRED_LIBS Core Gui Widgets Multimedia)
set(REQUIRED_LIBS_QUALIFIED Qt6::Core Qt6::Gui Qt6::Widgets Qt6::Multimedia)
#查找对应功能包
find_package(Qt${QT_VERSION} COMPONENTS ${REQUIRED_LIBS} REQUIRED)

# 链接库
link_directories(lib)

# 链接头文件
include_directories(include/robot
        include/haptic
        Application/Inc)
file(GLOB_RECURSE SOURCES "Application/Src/*.*")

add_executable(RM_QT_API main.cpp ${SOURCES}
        Application/Inc/connexts.h)

target_link_libraries(RM_QT_API ${REQUIRED_LIBS_QUALIFIED})
target_link_libraries(RM_QT_API
        dhdms64.lib
        drdms64.lib
        RM_Serviced.lib
)