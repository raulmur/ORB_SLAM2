# Find Pangolin (https://github.com/stevenlovegrove/Pangolin)
find_package(Pangolin 0.4 REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

add_executable(SimpleRepeatVideo main.cpp)
target_link_libraries(SimpleRepeatVideo ${Pangolin_LIBRARIES})
