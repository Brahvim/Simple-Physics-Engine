cmake_minimum_required(VERSION 3.15)

project(SimplePhysicsEngine C)

add_library(
	SimplePhysicsEngine
	SHARED
	"main.c"
	"engine.c"
)

add_executable(
	SimplePhysicsEngineTest
	"main.c"
	"engine.c"
)
