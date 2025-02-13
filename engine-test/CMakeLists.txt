cmake_minimum_required(VERSION 3.15)

project(SimplePhysicsEngineTest C)

add_executable(
	SimplePhysicsEngineTest
	"main.c"
)

target_link_libraries(SimplePhysicsEngineTest PRIVATE SimplePhysicsEngine)
