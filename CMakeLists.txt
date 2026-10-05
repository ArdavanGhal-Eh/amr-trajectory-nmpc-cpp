cmake_minimum_required(VERSION 3.16)
project(AmrTrajectoryNmpc VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall -Wextra")

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(include ${EIGEN3_INCLUDE_DIRS})

add_executable(amr_nmpc_solver
    src/main.cpp
    src/Model.cpp
    src/NmpcController.cpp
)

target_link_libraries(amr_nmpc_solver PRIVATE Eigen3::Eigen)
