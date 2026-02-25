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

# BulletEngine library
file(GLOB_RECURSE BULLET_ENGINE_SOURCES CONFIGURE_DEPENDS "${CMAKE_SOURCE_DIR}/src/*.cpp")
add_library(BulletEngine STATIC ${BULLET_ENGINE_SOURCES})
target_include_directories(BulletEngine PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(BulletEngine PUBLIC BulletRender BulletPhysics)

# samples/common
file(GLOB_RECURSE SAMPLES_COMMON_SOURCES CONFIGURE_DEPENDS "${CMAKE_SOURCE_DIR}/samples/common/*.cpp")
add_library(SamplesCommon STATIC ${SAMPLES_COMMON_SOURCES})
target_include_directories(SamplesCommon PUBLIC ${CMAKE_SOURCE_DIR}/samples)
target_link_libraries(SamplesCommon PUBLIC BulletEngine)

# function to generate one sample
function(add_sample SAMPLE_NAME SAMPLE_DIR)
    file(GLOB_RECURSE SAMPLE_SOURCES CONFIGURE_DEPENDS "${SAMPLE_DIR}/*.cpp")

    add_executable(${SAMPLE_NAME} ${SAMPLE_SOURCES})
    target_include_directories(${SAMPLE_NAME} PRIVATE ${SAMPLE_DIR})
    target_include_directories(${SAMPLE_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/samples)
    target_link_libraries(${SAMPLE_NAME} PRIVATE SamplesCommon)

    # copy resources
    add_custom_command(TARGET ${SAMPLE_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
                ${CMAKE_CURRENT_SOURCE_DIR}/BulletRender/assets
                $<TARGET_FILE_DIR:${SAMPLE_NAME}>/assets
        COMMENT "Copying BulletRender assets for ${SAMPLE_NAME}..."
    )

    add_custom_command(TARGET ${SAMPLE_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
                ${CMAKE_CURRENT_SOURCE_DIR}/BulletPhysics/assets
                $<TARGET_FILE_DIR:${SAMPLE_NAME}>/assets
        COMMENT "Copying BulletPhysics assets for ${SAMPLE_NAME}..."
    )

    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/assets)
        add_custom_command(TARGET ${SAMPLE_NAME} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory
                    ${CMAKE_CURRENT_SOURCE_DIR}/assets
                    $<TARGET_FILE_DIR:${SAMPLE_NAME}>/assets
            COMMENT "Copying BulletEngine assets for ${SAMPLE_NAME}..."
        )
    endif()
endfunction()

# add all samples
add_sample(ProjectileBasic "${CMAKE_SOURCE_DIR}/samples/projectile-basic")
add_sample(DragComparison "${CMAKE_SOURCE_DIR}/samples/drag-comparison")
add_sample(TerminalBallistics "${CMAKE_SOURCE_DIR}/samples/terminal-ballistics")