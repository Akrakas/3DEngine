cmake_minimum_required(VERSION 2.8)
project( 3DEngine )

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Build/)
set(SOURCE_FILES ${PROJECT_SOURCE_DIR}/Sources/3DEngine_Testbench_Main.cpp ${PROJECT_SOURCE_DIR}/Sources/engine.cpp ${PROJECT_SOURCE_DIR}/Sources/world_objects.cpp ${PROJECT_SOURCE_DIR}/Sources/physics.cpp ${PROJECT_SOURCE_DIR}/Sources/geometric_shapes.cpp ${PROJECT_SOURCE_DIR}/Sources/objloader.cpp)

set(EXECUTABLE 3DEngine_Testbench)

# add modules directory
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/Modules")

########################
# find packages#########
########################
#OPENCV
find_package( OpenCV REQUIRED )
#OPENGL
find_package( OpenGL REQUIRED )
#OPENGL GLEW
find_package( GLEW REQUIRED )
#OPENGL GLFW
find_package( GLFW REQUIRED )

set(PROJECT_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/Includes/ ${OpenCV_INCLUDE_DIRS} ${OPENGL_INCLUDE_DIR} ${GLFW_INCLUDE_DIR})
set(PROJECT_LIBRARIES ${OpenCV_LIBS} ${OPENGL_LIBRARIES} GLEW::GLEW ${GLFW_LIBRARY}) #glew why

MESSAGE( STATUS "PROJECT_INCLUDE_DIR: " ${PROJECT_INCLUDE_DIR} )
MESSAGE( STATUS "PROJECT_LIBRARIES: " ${PROJECT_LIBRARIES} )
include_directories( ${PROJECT_INCLUDE_DIR} )
add_executable(${EXECUTABLE} ${SOURCE_FILES})
target_link_libraries(${EXECUTABLE} ${PROJECT_LIBRARIES})