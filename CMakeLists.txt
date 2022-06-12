cmake_minimum_required(VERSION 3.9)

project(hybrid_a_star)

set(CMAKE_CXX_STANDARD 17)

aux_source_directory(. DIR_SRCS)
  
add_subdirectory(src)  #包含的子文件夹

add_executable(test main.cc)


aux_source_directory(. source_list)

target_link_libraries(test BasicFunciton)

#add_executable(test main.cc)