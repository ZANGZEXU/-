aux_source_directory(src MY_LIB_RMATH)
add_library(rmath SHARED ${MY_LIB_RMATH})
target_include_directories(rmath PUBLIC include)
target_include_directories(rmath PUBLIC common/singleton/include)


find_package(OpenCV REQUIRED)
target_include_directories(rmath PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(rmath ${OpenCV_LIBS})