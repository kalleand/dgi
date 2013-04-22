# Top-Level CMakeList.txt

SET(CMAKE_CXX_FLAGS "-std=c++0x")

cmake_minimum_required (VERSION 2.6)
project ( RayTracing )

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

