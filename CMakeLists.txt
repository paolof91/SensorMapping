cmake_minimum_required(VERSION 2.8)
project( SensorMapping )
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
set(SOURCES OccupancyGridMapper.cpp OccupancyGrid.cpp RectangleDetector.cpp)
add_executable( SensorMapping SensorMapping.cpp "${SOURCES}")
target_link_libraries( SensorMapping ${OpenCV_LIBS} )

# TESTS
add_custom_target(tests)
add_executable(__unit_tests__SensorMapping __unit_tests.cpp "${SOURCES}") # Note: without 'ALL'
target_link_libraries( __unit_tests__SensorMapping ${OpenCV_LIBS} )
add_dependencies(tests __unit_tests__SensorMapping)