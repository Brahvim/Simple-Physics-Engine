cmake_minimum_required(VERSION 3.15)

project(PortedSimplePhysicsEngine)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

include_directories(
	${CMAKE_SOURCE_DIR}/include
)

add_subdirectory("engine")
add_subdirectory("engine-test")
add_subdirectory("jni-connector")
