aux_source_directory(src MY_LIB_A1)
aux_source_directory(test MY_LIB_A1_TEST)

add_library(assembly1 SHARED ${MY_LIB_A1} ${MY_LIB_A1_TEST})
target_include_directories(assembly1 PUBLIC include)


find_package(OpenCV REQUIRED)
target_include_directories(assembly1 PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(assembly1 ${OpenCV_LIBS})