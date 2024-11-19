cmake_minimum_required(VERSION 3.0)
project(swarm_final_ggn)

set(CMAKE_CXX_STANDARD 14)
# Set build type to Debug
set(CMAKE_BUILD_TYPE Debug)

# Add debug flags
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")


include_directories(include)

# Add libraries
add_library(load_map 
    src/load_map.cpp
)

add_library(GlobalPlanner 
    src/GlobalPlanner.cpp
)

# Convert main_planner to a library
add_library(main_planner
    src/main_planner.cpp
)

target_include_directories(main_planner 
    PRIVATE ${CMAKE_SOURCE_DIR}/include
)

# Link dependencies to main_planner library
target_link_libraries(main_planner 
    PRIVATE load_map 
    PRIVATE GlobalPlanner
)

# Define paths for main_planner
target_compile_definitions(main_planner 
    PRIVATE MAPS_DIR="${CMAKE_SOURCE_DIR}/maps"
    PRIVATE OUTPUT_DIR="${CMAKE_SOURCE_DIR}/output"
)

# Enable compilation database
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Add the test executable
add_executable(run_test
    src/runtest.cpp
)

# Set include directories for run_test
target_include_directories(run_test 
    PRIVATE ${CMAKE_SOURCE_DIR}/include
)

# Link run_test with main_planner and its dependencies
target_link_libraries(run_test 
    PRIVATE main_planner
    PRIVATE load_map
    PRIVATE GlobalPlanner
)

# Define paths for run_test
target_compile_definitions(run_test 
    PRIVATE MAPS_DIR="${CMAKE_SOURCE_DIR}/maps"
    PRIVATE OUTPUT_DIR="${CMAKE_SOURCE_DIR}/output"
)