cmake_minimum_required(VERSION 3.15)

project(SimplePhysicsEngineJniDriver)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)
set(JAVA_HOME "/home/brahvim/Programs/java/jdk-17.0.7+7/")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fno-stack-protector -z noexecstack")

include_directories(
	${JAVA_HOME}/include
	${JAVA_HOME}/include/linux
	${CMAKE_SOURCE_DIR}/../include/
)

# Build JNI headers before build:
execute_process(
	RESULT_VARIABLE result_jnigen
	OUTPUT_VARIABLE output_jnigen
	ERROR_VARIABLE error_output_jnigen
	WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
	COMMAND bash ${CMAKE_SOURCE_DIR}/jnigen.sh
)

# *Did you?:*
if(NOT result_jnigen EQUAL 0)
	message(FATAL_ERROR "JNI headers build failed!:\n${error_output_jnigen}")
endif()
message("Built JNI headers successfully.")

add_library(
	SimplePhysicsEngineJniDriver
	SHARED
	"jni.c"
)

# Run this script post-build to inject the generated library into the Java project:
add_custom_command(
		TARGET SimplePhysicsEngineJniDriver
        POST_BUILD
        COMMAND bash ${CMAKE_SOURCE_DIR}/javainject.sh
)