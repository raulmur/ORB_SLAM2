file(GLOB SEGMENTATION_SRC ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
file(GLOB SEGMENTATION_INCLUDES ${CMAKE_SOURCE_DIR}/include/segmentation/*.h)

add_library(segmentation SHARED ${SEGMENTATION_SRC})
target_include_directories(segmentation PUBLIC ${CMAKE_SOURCE_DIR}/include/segmentation/)
target_link_libraries(segmentation PUBLIC ${OpenCV_LIBS})