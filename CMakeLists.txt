# CMakeLists.txt

cmake_minimum_required(VERSION 3.22)
project(BulletEngine)

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

# connect BulletRender as library
set(BULLET_RENDER_BUILD_APP OFF CACHE BOOL "" FORCE)    # disable demo compilation within BulletRender
add_subdirectory(BulletRender)

# connect BulletPhysics library
add_subdirectory(BulletPhysics)

# BulletEngine application
file(GLOB_RECURSE BULLET_ENGINE_SOURCES CONFIGURE_DEPENDS "${CMAKE_SOURCE_DIR}/src/*.cpp")
add_executable(BulletEngine ${CMAKE_SOURCE_DIR}/main.cpp ${BULLET_ENGINE_SOURCES})
target_include_directories(BulletEngine PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(BulletEngine PRIVATE BulletRenderLib BulletPhysicsLib)

# copy resources
add_custom_command(TARGET BulletEngine POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_CURRENT_SOURCE_DIR}/BulletRender/assets
            $<TARGET_FILE_DIR:BulletEngine>/assets
    COMMENT "Copying BulletRender assets..."
)

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/assets)
    add_custom_command(TARGET BulletEngine POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
                ${CMAKE_CURRENT_SOURCE_DIR}/assets
                $<TARGET_FILE_DIR:BulletEngine>/assets
        COMMENT "Copying BulletEngine assets..."
    )
endif()
