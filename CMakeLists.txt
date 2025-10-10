# CMakeLists.txt

cmake_minimum_required(VERSION 3.22)
project(luchengine)

set(CMAKE_CXX_STANDARD 20)

# address sanitizer
option(SANITIZE "enable AddressSanitizer with extended checks" OFF)

if(SANITIZE)
    message(STATUS "AddressSanitizer enabled")
    set(ASAN_FLAGS "-fsanitize=address -fno-omit-frame-pointer -g")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ASAN_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ASAN_FLAGS}")

    # embed runtime ASAN options into binary
    add_compile_definitions(ASAN_OPTIONS="detect_leaks=1:strict_string_checks=1:check_initialization_order=1:detect_stack_use_after_return=1:detect_container_overflow=1:abort_on_error=1")
endif()

# disable demo compilation within luchrender
set(LUCHRENDER_BUILD_APP OFF CACHE BOOL "" FORCE)

# connect luchrender as library
add_subdirectory(luchrender)

# connect luchphysic as library
add_subdirectory(luchphysic)

# collect all source files recursively, excluding build directory
file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS "${CMAKE_SOURCE_DIR}/*.cpp")
list(FILTER SOURCES EXCLUDE REGEX ".*/.*build.*/.*")
list(FILTER SOURCES EXCLUDE REGEX ".*/luchrender/.*")
list(FILTER SOURCES EXCLUDE REGEX ".*/luchphysic/.*")

# creating executable
set(BIN_NAME demo)

add_executable(${BIN_NAME} ${SOURCES})
target_include_directories(${BIN_NAME} PRIVATE ${CMAKE_SOURCE_DIR})
target_include_directories(${BIN_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/src)

# linking luchrender
target_link_libraries(${BIN_NAME} PRIVATE luchrender)

# linking luchphysic
target_link_libraries(${BIN_NAME} PRIVATE luchphysic)

# copy resources to binary after compilation
add_custom_command(TARGET ${BIN_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_CURRENT_SOURCE_DIR}/assets
            $<TARGET_FILE_DIR:${BIN_NAME}>/assets
)