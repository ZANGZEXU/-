cmake_minimum_required(VERSION 3.0)
project(SouthTiger)


add_subdirectory(ArmorDescriptor)
add_subdirectory(ArmorParam)




##include_directories(test PUBLIC LightDescriptor/incldude)
add_executable(test main.cpp )



find_package(OpenCV REQUIRED)
target_include_directories(test PUBLIC ${OpenCV_INCLUDE_DIRS})
TARGET_LINK_LIBRARIES(test ${OpenCV_LIBS} ArmorParam ArmorDescriptor)