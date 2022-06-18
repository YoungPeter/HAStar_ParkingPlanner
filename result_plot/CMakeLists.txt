aux_source_directory(. DIR_LIB_SRCS)
set(CMAKE_CXX_STANDARD 17)

# Library target
add_library(ResultPlot INTERFACE)

target_compile_features(ResultPlot INTERFACE cxx_std_17)
# TODO: Use `Development.Embed` component when requiring cmake >= 3.18
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

if (Python3_FOUND)
message(STATUS "tests: ${Python3_EXECUTABLE}")
endif()

target_link_libraries(ResultPlot INTERFACE
  Python3::Python
  Python3::Module
)
find_package(Python3 COMPONENTS NumPy)
target_link_libraries(ResultPlot INTERFACE Python3::NumPy)