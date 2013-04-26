# Top-Level CMakeList.txt

SET(CMAKE_CXX_FLAGS "-O3 -std=c++0x")

cmake_minimum_required (VERSION 2.6)
project ( RayTracing )

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

add_executable( RayTracing ray_tracing.cpp)

find_package (SDL)

if ( NOT SDL_FOUND )
   message ( FATAL_ERROR "SDL not found!" )
endif ( NOT SDL_FOUND )

if(SDL_FOUND)
	include_directories(
		${SDL_INCLUDE_DIR}
		${PROJECT_SOURCE_DIR}/../glm
	)
	#link_libraries(${SDL_LIBRARY})
	target_link_libraries(RayTracing ${SDL_LIBRARY})
endif(SDL_FOUND)

