# Simple-Physics-Engine
A physics engine that aims to bring multiple kinds of solvers in an API "freedom"-friendly manner (granular control).  
I also aim to keep data structures cache-friendly, yes!...  

All implementation code is currently in a single [`library/engine.c`](https://github.com/brahvim/Simple-Physics-Engine/tree/master/library/engine.c) file!  
Might later be split according to solvers and their data.  

Building this code is as simple as installing OpenGL and GLFW3 development and library packages and running CMake!  
(My dev machine runs Debian, you may want to keep that in mind!)  

Currently, the data layout is a bit incorrect.  
While all bodies' IDs are in the domain of the translation parameters, `SpContext::masses` is *not*.  
Will correct this possibly in the next commit.  
