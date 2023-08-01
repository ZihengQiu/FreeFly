aux_source_directory(./Inc DIR_LIB_SRCS)
add_library(FWLib ${DIR_LIB_SRCS})
target_include_directories(FWLib ${CMAKE_CURRENT_SOURCE_DIR})
