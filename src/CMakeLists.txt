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

# Add rbf.cpp

add_library(rbf rbf.cpp)

target_include_directories(rbf
    PUBLIC ${CMAKE_SOURCE_DIR}/libraries/eigen
)

target_link_directories(rbf
    PUBLIC ${CMAKE_SOURCE_DIR}/libraries/eigen
)

target_link_libraries(rbf eigen)

