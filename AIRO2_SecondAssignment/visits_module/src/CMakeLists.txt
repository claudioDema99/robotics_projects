project(Test)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}
                      ${CMAKE_CURRENT_SOURCE_DIR}
                      ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

set(CMAKE_CXX_FLAGS "-Wall -std=gnu++11 ${CMAKE_CXX_FLAGS}")

add_library(Visits SHARED VisitSolver.cpp ExternalSolver.cpp)
add_executable(VisitMain main.cpp)

target_link_libraries(VisitMain Visits)