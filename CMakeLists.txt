cmake_minimum_required(VERSION 3.15)

project(SimplePhysicsEngine)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)
set(CMAKE_C_FLAGS
    #
    # Memory bugs only:
    # "${CMAKE_C_FLAGS} -g -O3 -fsanitize=address -fno-omit-frame-pointer"
    #
    # Tested on Compiler Explorer:
    "${CMAKE_C_FLAGS} -O3 -march=native -ffast-math -funroll-loops -mavx2 -mprefer-vector-width=256 \
	-ftree-vectorize -fopt-info-vec-optimized "
    # Parameters for SoAs test w/ AVX intrins. Didn't run on my Ryzen 5 5600H:
    #
    # "${CMAKE_C_FLAGS} -O3 -march=native -ffast-math -funroll-loops \ -msse4.2
    # -mprefer-vector-width=128 \ -mtune=znver3 -mabm -mlwp -mno-fma4 \
    # -mno-avx256-split-unaligned-load -mfma \ -ftree-vectorize
    # -fopt-info-vec-optimized"
    #
    # The usual, speed: "${CMAKE_C_FLAGS} -g -O3 -march=native -Wall -Wextra
    # -Wno-unknown-pragmas -Wno-unused-parameter \ -fopt-info-vec"
    #
    # The usual, bugs: "${CMAKE_C_FLAGS} -g -O3 -march=native -Wall -Wextra
    # -Wno-unknown-pragmas -Wno-unused-parameter \ -fopt-info-vec
    # -fsanitize=address -fno-omit-frame-pointer"
    #
    # All the speed: "${CMAKE_C_FLAGS} -g -O3 -march=native -Wall -Wextra
    # -Wno-unknown-pragmas -Wno-unused-parameter \ -fopt-info-vec -ffast-math
    # -funroll-loops -mavx2 -mprefer-vector-width=256 -ftree-vectorize \
    # -fopt-info-vec-optimized"
    #
    # All the speed, bug detection bundled: "${CMAKE_C_FLAGS} -g -O3
    # -march=native -Wall -Wextra -Wno-unknown-pragmas -Wno-unused-parameter \
    # -fopt-info-vec -ffast-math -funroll-loops -mavx2 -mprefer-vector-width=256
    # -ftree-vectorize \ -fopt-info-vec-optimized \ -fsanitize=address
    # -fno-omit-frame-pointer"
)

include_directories(${CMAKE_SOURCE_DIR}/include)

add_subdirectory("library")
add_subdirectory("test")
