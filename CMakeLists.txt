cmake_minimum_required(VERSION 3.16)
project(slide_controller C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Werror)

add_library(slide STATIC src/slide.c)
target_include_directories(slide PUBLIC include)
target_link_libraries(slide PUBLIC m)

add_library(slide_app STATIC src/app.c src/proto.c src/crc8.c src/ringbuf.c)
target_link_libraries(slide_app PUBLIC slide)

add_library(slide_sim_lib STATIC sim/sim.c)
target_include_directories(slide_sim_lib PUBLIC sim)
target_link_libraries(slide_sim_lib PUBLIC slide)

add_executable(slide_sim sim/main.c)
target_link_libraries(slide_sim PRIVATE slide_sim_lib)

enable_testing()
add_executable(test_slide tests/test_slide.c)
target_link_libraries(test_slide PRIVATE slide_sim_lib)
add_test(NAME test_slide COMMAND test_slide)

add_executable(test_app tests/test_app.c)
target_link_libraries(test_app PRIVATE slide_app)
add_test(NAME test_app COMMAND test_app)
