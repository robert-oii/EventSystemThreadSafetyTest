cmake_minimum_required(VERSION 3.14.0 FATAL_ERROR)

project(events_thread_test_proj)

option(SANITIZE "Build with sanitizers" OFF)

add_executable(events_thread_test)

target_sources(events_thread_test PRIVATE main.cpp)

target_link_libraries(events_thread_test PRIVATE pthread)

if (SANITIZE)
    target_compile_options(events_thread_test
        PRIVATE
            -fsanitize=thread
            -fno-omit-frame-pointer
    )
    target_link_libraries(events_thread_test
        PRIVATE
            -fsanitize=thread
            -fno-omit-frame-pointer
    )
endif()
