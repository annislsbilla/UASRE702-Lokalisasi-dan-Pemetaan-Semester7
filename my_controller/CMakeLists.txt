cmake_minimum_required(VERSION 3.20.0)



# Setup the project.
# Its name is defined to be the controller directory name.



get_filename_component(PROJECT ${CMAKE_SOURCE_DIR} NAME)
project(${PROJECT})


find_package(OpenCV REQUIRED)
find_package(Python3 REQUIRED)
include_directories(${Python3_INCLUDE_DIRS})
target_link_libraries(your_project_name ${Python3_LIBRARIES})
target_link_libraries(my_controller PRIVATE Python3::Python)

include_directories(/usr/include/eigen3)
include_directories(${OpenCV_INCLUDE_DIRS})
# Get C or C++ sources in the current directory (only).
file(GLOB C_SOURCES *.c)
file(GLOB CPP_SOURCES *.cpp)
set(SOURCES ${C_SOURCES} ${CPP_SOURCES})

# Set the  Webots home path (change it according to your installation method)
set(WEBOTS_HOME "/usr/local/webots")
#set(WEBOTS_HOME "/snap/webots/current/usr/share/webots")

# Link with the Webots controller library.
link_directories($ENV{WEBOTS_HOME}/lib/controller)
set (LIBRARIES m ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX} ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX})
include_directories($ENV{WEBOTS_HOME}/include/controller/c $ENV{WEBOTS_HOME}/include/controller/cpp)

# Setup the target executable.
add_executable(${PROJECT} ${SOURCES})
target_link_libraries(${PROJECT} ${LIBRARIES} ${OpenCV_LIBS})

# Copy the target executable at the right location.
add_custom_command(TARGET ${PROJECT} POST_BUILD COMMAND ${CMAKE_COMMAND} -E
  copy ${CMAKE_BINARY_DIR}/${PROJECT} ${CMAKE_SOURCE_DIR}
)
