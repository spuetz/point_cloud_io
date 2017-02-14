cmake_minimum_required(VERSION 2.8.3)
project(point_cloud_io)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES point_cloud_io
  CATKIN_DEPENDS roscpp sensor_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(point_cloud_io
  src/point_cloud_io.cpp
)

add_dependencies(point_cloud_io ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(point_cloud_io_reader 
  src/reader.cpp
  #src/point_cloud_io.cpp  
)
add_executable(point_cloud_io_writer src/writer.cpp)

add_dependencies(point_cloud_io_reader ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(point_cloud_io_writer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(point_cloud_io_reader
  point_cloud_io
  ${catkin_LIBRARIES}
)

target_link_libraries(point_cloud_io_writer
  point_cloud_io
  ${catkin_LIBRARIES}
)

install(TARGETS point_cloud_io point_cloud_io_reader point_cloud_io_writer
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch"
)
