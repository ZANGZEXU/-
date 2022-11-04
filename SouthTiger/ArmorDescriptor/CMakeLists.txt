aux_source_directory(src DIR_LIB_SRCS)
add_library(ArmorDescriptor SHARED ${DIR_LIB_SRCS})

target_include_directories(ArmorDescriptor PUBLIC include)
target_include_directories(ArmorDescriptor PUBLIC include/LightDescriptor/include)
find_package(OpenCV REQUIRED)
target_include_directories(ArmorDescriptor PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(ArmorDescriptor ${OpenCV_LIBS} ArmorParam)  