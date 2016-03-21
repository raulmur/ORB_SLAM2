cmake_minimum_required(VERSION 2.8)
project(dbow2)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

PROTOBUF_CATKIN_GENERATE_CPP(PROTO_SRCS PROTO_HDRS proto/node.proto
                                                   proto/vocabulary.proto
                                                   proto/word.proto)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

set(SRCS_DBOW2
  src/BowVector.cpp
  src/FORB.cpp      
  src/FeatureVector.cpp
  src/ScoringObject.cpp
  src/TemplatedVocabulary.cpp)
set(SRCS_DUTILS
  src/Random.cpp
  src/Timestamp.cpp)

add_definitions("-std=c++11")

cs_add_library(${PROJECT_NAME} ${SRCS_DBOW2} ${SRCS_DUTILS} ${PROTO_SRCS})

cs_add_executable(voc_to_proto src/voc_to_proto.cpp)
target_link_libraries(voc_to_proto ${PROJECT_NAME})

cs_install()
# we need to add the current binary dir to include dirs due to protobuf
cs_export(INCLUDE_DIRS include ${CMAKE_CURRENT_BINARY_DIR})