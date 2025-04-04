cmake_minimum_required(VERSION 3.15)

project(SimplePhysicsEngineTest C)

find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

add_executable(SimplePhysicsEngineTest "main.c")

target_link_libraries(
  SimplePhysicsEngineTest PRIVATE SimplePhysicsEngineLibrary m OpenGL::GL
                                  /usr/lib/x86_64-linux-gnu/libglfw.so)
