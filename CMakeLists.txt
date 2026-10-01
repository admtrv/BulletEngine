# CMakeLists.txt

cmake_minimum_required(VERSION 3.22)

# gcc 16 is the currently supported compiler for c++26 reflection
find_program(GCC_16 gcc-16)
find_program(GXX_16 g++-16)

if(NOT GCC_16 OR NOT GXX_16)
    message(FATAL_ERROR "BulletEngine needs gcc 16 for c++26 reflection, install gcc-16 and g++-16")
endif()

set(CMAKE_C_COMPILER "${GCC_16}" CACHE FILEPATH "" FORCE)
set(CMAKE_CXX_COMPILER "${GXX_16}" CACHE FILEPATH "" FORCE)

project(BulletEngine)

if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 16)
    message(FATAL_ERROR "BulletEngine needs gcc 16 or newer, found ${CMAKE_CXX_COMPILER_VERSION}")
endif()

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

# connect BulletRender as library
set(BULLET_RENDER_BUILD_APP OFF CACHE BOOL "" FORCE)    # disable building BulletRender as standalone app
add_subdirectory(BulletRender)

# connect BulletPhysics library
add_subdirectory(BulletPhysics)

# lua library
file(GLOB LUA_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/external/lua/*.c)
list(REMOVE_ITEM LUA_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/external/lua/lua.c          # standalone interpreter
    ${CMAKE_CURRENT_SOURCE_DIR}/external/lua/onelua.c)      # every source in one file

add_library(lua STATIC ${LUA_SOURCES})
target_include_directories(lua PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/external/lua)
target_compile_definitions(lua PUBLIC LUA_USE_LINUX)

# BulletEngine application
file(GLOB_RECURSE BULLET_ENGINE_SOURCES CONFIGURE_DEPENDS "${CMAKE_SOURCE_DIR}/src/*.cpp")
add_executable(BulletEngine ${CMAKE_SOURCE_DIR}/main.cpp ${BULLET_ENGINE_SOURCES})
target_include_directories(BulletEngine PRIVATE ${CMAKE_SOURCE_DIR}/src ${CMAKE_SOURCE_DIR}/external/sol2/include)
target_link_libraries(BulletEngine PRIVATE BulletRenderLib BulletPhysicsLib lua)

# c++26 reflection
target_compile_features(BulletEngine PRIVATE cxx_std_26)
target_compile_options(BulletEngine PRIVATE -freflection)

# editor resources
foreach(RESOURCE shaders fonts)
    add_custom_command(TARGET BulletEngine POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
                ${CMAKE_CURRENT_SOURCE_DIR}/BulletRender/assets/${RESOURCE}
                $<TARGET_FILE_DIR:BulletEngine>/assets/${RESOURCE}
        COMMENT "Copying ${RESOURCE}..."
    )
endforeach()
