cmake_minimum_required(VERSION 3.0.0)
project(ControlSoftware VERSION 0.1.0)

include(CTest)
enable_testing()

add_executable(ControlSoftware main.cpp)

add_subdirectory(src)
add_subdirectory(libraries/eigen)
add_subdirectory(libraries/casadi)

target_include_directories(${PROJECT_NAME}
    PUBLIC src
    PUBLIC libraries/eigen
    PUBLIC libraries/casadi
)

target_link_directories(${PROJECT_NAME}
    PUBLIC src
    PUBLIC libraries/eigen
    PUBLIC libraries/casadi
)

target_link_libraries(${PROJECT_NAME} eigen casadi dynamics rbf)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)