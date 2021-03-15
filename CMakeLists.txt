project(cbs)

cmake_minimum_required (VERSION 2.6)

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE "RELEASE")
ENDIF()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

include_directories("inc")
file(GLOB SOURCES "src/*.cpp")
add_executable(cbs ${SOURCES})

# Find Boost
find_package(Boost REQUIRED COMPONENTS program_options system filesystem)
include_directories( ${Boost_INCLUDE_DIRS} )
target_link_libraries(cbs ${Boost_LIBRARIES})

