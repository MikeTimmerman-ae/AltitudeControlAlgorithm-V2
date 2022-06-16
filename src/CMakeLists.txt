##
##     Filename:  src/CMakeLists.txt
##     Author:    Mike Timmerman
##     Version:   4.0
##     Date:      2022
##

# Add dynamics.cpp

add_library(dynamics dynamics.cpp)

target_include_directories(dynamics
    PUBLIC ${CMAKE_SOURCE_DIR}/libraries/eigen
)

target_link_directories(dynamics
    PUBLIC ${CMAKE_SOURCE_DIR}/libraries/eigen
)

target_link_libraries(dynamics eigen)


# Add controller.cpp

add_library(controller controller.cpp)

target_include_directories(controller
    PUBLIC ${CMAKE_SOURCE_DIR}/libraries/eigen
)

target_link_directories(controller
    PUBLIC ${CMAKE_SOURCE_DIR}/libraries/eigen
)

target_link_libraries(controller eigen)


# Add simulator.cpp

add_library(simulator simulator.cpp)

target_include_directories(simulator
    PUBLIC ${CMAKE_SOURCE_DIR}/libraries/eigen
)

target_link_directories(simulator
    PUBLIC ${CMAKE_SOURCE_DIR}/libraries/eigen
)

target_link_libraries(simulator eigen)


# Add saturator.cpp

add_library(saturator saturator.cpp)

target_include_directories(saturator
    PUBLIC ${CMAKE_SOURCE_DIR}/libraries/eigen
)

target_link_directories(saturator
    PUBLIC ${CMAKE_SOURCE_DIR}/libraries/eigen
)

target_link_libraries(saturator eigen)


# Add helpers.cpp

add_library(helpers helpers.cpp)

target_include_directories(helpers
    PUBLIC ${CMAKE_SOURCE_DIR}/libraries/eigen
)

target_link_directories(helpers
    PUBLIC ${CMAKE_SOURCE_DIR}/libraries/eigen
)

target_link_libraries(helpers eigen)
