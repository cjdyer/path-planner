cmake_minimum_required(VERSION 3.8)
project(path_planner)

add_subdirectory(lib/SDL)
add_subdirectory(lib/yaml-cpp)

include_directories(include)

set(sources 
    src/main.cpp
    src/config.cpp
    src/path_planner.cpp
    src/astar.cpp
    src/dijkstra.cpp
    src/bfs.cpp
    )

add_executable(${PROJECT_NAME}
    ${sources}
)

target_link_libraries(${PROJECT_NAME} PRIVATE yaml-cpp SDL3::SDL3)

configure_file(config.yaml ${CMAKE_CURRENT_BINARY_DIR}/config.yaml COPYONLY)