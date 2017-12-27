cmake_minimum_required(VERSION 2.8)
project( 3DEngine )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Build/)
set(SOURCE_FILES ${PROJECT_SOURCE_DIR}/Sources/3DEngine_Testbench_Main.cpp ${PROJECT_SOURCE_DIR}/Sources/engine.cpp ${PROJECT_SOURCE_DIR}/Sources/world_objects.cpp ${PROJECT_SOURCE_DIR}/Sources/physics.cpp ${PROJECT_SOURCE_DIR}/Sources/geometric_shapes.cpp)

set(EXECUTABLE 3DEngine_Testbench)
set(PROJECT_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/Includes/)
# find packages
find_package( OpenCV REQUIRED )
set(PROJECT_INCLUDES ${OpenCV_INCLUDE_DIRS})
set(PROJECT_LIBRARIES ${OpenCV_LIBS})

include_directories( ${PROJECT_INCLUDE_DIR} ${PROJECT_INCLUDES})
add_executable(${EXECUTABLE} ${SOURCE_FILES})
target_link_libraries(${EXECUTABLE} ${PROJECT_LIBRARIES})