cmake_minimum_required(VERSION 3.17)
project(API_QT)

set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_PREFIX_PATH "D:/software/Qt/6.7.0/mingw_64")
set(CMAKE_AUTOMOC ON)

set(QT_VERSION 6)

#链接必要库文件
set(REQUIRED_LIBS Core Gui Widgets Multimedia)
set(REQUIRED_LIBS_QUALIFIED Qt6::Core Qt6::Gui Qt6::Widgets Qt6::Multimedia)

#查找对应功能包
find_package(Qt${QT_VERSION} COMPONENTS ${REQUIRED_LIBS} REQUIRED)

link_directories(lib)
include_directories(include ./Eigen)
file(GLOB_RECURSE SOURCES "autocenter/*.*")

add_executable(CLI main.cpp ${SOURCES}
        autocenter/autocenter.cpp
        autocenter/autocenter.cpp)

target_link_libraries(CLI dhdms.lib dhdms64.lib drdms.lib drdms64.lib)
target_link_libraries(CLI ${REQUIRED_LIBS_QUALIFIED})
