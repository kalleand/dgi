# Top-Level CMakeList.txt

SET(CMAKE_CXX_FLAGS "-std=c++0x")

cmake_minimum_required (VERSION 2.6)

project(Gradient)

ADD_LIBRARY(Timer SHARED timer.c)

add_executable(Gradient gradient.cpp)

add_executable(StarField starfield.cpp)

find_package (SDL)

if ( NOT SDL_FOUND )
   message ( FATAL_ERROR "SDL not found!" )
endif ( NOT SDL_FOUND )

if(SDL_FOUND)
	include_directories(
		${SDL_INCLUDE_DIR}
		${PROJECT_SOURCE_DIR}/../glm
	)

	target_link_libraries(Gradient ${SDL_LIBRARY})
	target_link_libraries(StarField ${SDL_LIBRARY} Timer)
endif(SDL_FOUND)

