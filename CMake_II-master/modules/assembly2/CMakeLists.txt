aux_source_directory(src MY_LIB_A2)
add_library(assembly2 SHARED ${MY_LIB_A2})
target_include_directories(assembly2 PUBLIC include)

find_package(OpenCV REQUIRED)
target_include_directories(assembly2 PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(assembly2 ${OpenCV_LIBS})