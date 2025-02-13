cmake_minimum_required(VERSION 3.15)

project(SimplePhysicsEngine)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

include_directories(
	${CMAKE_SOURCE_DIR}/include/
)

add_subdirectory("jni")

add_library(
	SimplePhysicsEngine
	SHARED
	"main.c"
	"engine.c"
)
