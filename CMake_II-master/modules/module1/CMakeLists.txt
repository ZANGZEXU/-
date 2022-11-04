aux_source_directory(src/module1a MY_LIB_M1A)
aux_source_directory(src/module1b MY_LIB_M1B)
add_library(module1 SHARED ${MY_LIB_M1A} ${MY_LIB_M1B})
target_include_directories(module1 PUBLIC include)
target_link_libraries(module1 assembly2)

find_package(OpenCV REQUIRED)
target_include_directories(module1 PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(module1 ${OpenCV_LIBS})