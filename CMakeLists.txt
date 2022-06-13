cmake_minimum_required(VERSION 3.9)

project(hybrid_a_star LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

aux_source_directory(. DIR_SRCS)
  
add_subdirectory(src)  

add_subdirectory(result_plot) 

add_executable(test main.cc)

aux_source_directory(. source_list)

target_link_libraries(test BasicFunciton ResultPlot) 

set_target_properties(test PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")