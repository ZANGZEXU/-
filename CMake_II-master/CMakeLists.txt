cmake_minimum_required(VERSION 3.10)
project(Test)
# # # # 以下内容禁止修改 # # # #
option(BUILD_A "build modules a" ON)
option(BUILD_B "build modules b" OFF)
option(BUILD_TESTS "build tests" OFF)

if(BUILD_TESTS)
    enable_testing()
endif()

add_subdirectory(modules)
add_subdirectory(common)
# 小提示: module2 需要用到 pthread，即 target_link_libraries(xxx pthread)
# # # # 以上内容禁止修改 # # # #


add_executable(demo server.cpp client.cpp)
find_package(OpenCV REQUIRED)
target_include_directories(demo PUBLIC modules/module1/include)
target_include_directories(demo PUBLIC common/singleton/include)
target_include_directories(demo PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(
    demo  PUBLIC
    ${OpenCV_LIBS} module1a module1b module2 assembly1 assembly2 rmath)
