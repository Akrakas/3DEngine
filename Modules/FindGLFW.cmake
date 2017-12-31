# Locates the GLFW library and include directories.

include(FindPackageHandleStandardArgs)
unset(GLFW_FOUND)

find_path(GLFW_INCLUDE_DIR
	NAMES
	GLFW
	HINTS
	/usr/include/)

find_library(GLFW_LIBRARY 
	NAMES 
	glfw
	HINTS
	/usr/lib
	/usr/local/lib)

# set GLFW_FOUND
find_package_handle_standard_args(GLFW DEFAULT_MSG GLFW_INCLUDE_DIR GLFW_LIBRARY)

# set external variables for usage in CMakeLists.txt
if(GLFW_FOUND)
	set(GLFW_LIBRARIES ${GLFW_LIBRARY})
	set(GLFW_INCLUDE_DIRS ${GLFW_INCLUDE_DIR})
endif()

# hide locals from GUI
mark_as_advanced(GLFW_INCLUDE_DIR GLFW_LIBRARY)