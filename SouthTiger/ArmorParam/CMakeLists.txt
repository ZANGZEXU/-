aux_source_directory(src DIR_AP)
add_library(ArmorParam SHARED ${DIR_AP})
target_include_directories(ArmorParam  PUBLIC include)



find_package(OpenCV REQUIRED)
target_include_directories(ArmorParam PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(ArmorParam ${OpenCV_LIBS})