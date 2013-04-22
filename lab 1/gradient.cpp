// Introduction lab that covers:
// * C++
// * SDL
// * 2D graphics
// * Plotting pixels
// * Video memory
// * Color representation
// * Linear interpolation
// * glm::vec3 and std::vector

#include "SDL.h"
#include <iostream>
#include <glm/glm.hpp>
#include <vector>
#include "SDLauxiliary.h"

using std :: vector;
using glm :: vec3;

// --------------------------------------------------------
// GLOBAL VARIABLES

const int SCREEN_WIDTH = 640;
const int SCREEN_HEIGHT = 480;
SDL_Surface * screen;

// --------------------------------------------------------
// FUNCTION DECLARATIONS

void Draw();
template<typename T>
void Interpolate(const T & a, const T & b, vector<T> & result);

// --------------------------------------------------------
// FUNCTION DEFINITIONS

int main(int argc, char * argv[]) {
	screen = InitializeSDL(SCREEN_WIDTH, SCREEN_HEIGHT);
	while (NoQuitMessageSDL())
		Draw();

	SDL_SaveBMP(screen, "screenshot.bmp");
	return EXIT_SUCCESS;
}

void Draw() {
	vec3 topLeft(1, 0, 0); // red
	vec3 topRight(0, 0, 1); // blue
	vec3 bottomLeft(0, 1, 0); // green
	vec3 bottomRight(1, 1, 0); // yellow

	vector<vec3> leftSide(SCREEN_HEIGHT);
	vector<vec3> rightSide(SCREEN_HEIGHT);
	Interpolate(topLeft, bottomLeft, leftSide);
	Interpolate(topRight, bottomRight, rightSide);

	for (int y = 0; y < SCREEN_HEIGHT; ++y) {
		vector<vec3> rowColors(SCREEN_WIDTH);
		Interpolate(leftSide[y], rightSide[y], rowColors);

		for (int x = 0; x < SCREEN_WIDTH; ++x) {
			PutPixelSDL(screen, x, y, rowColors[x]);
		}
	}

	if (SDL_MUSTLOCK(screen))
		SDL_UnlockSurface(screen);

	SDL_UpdateRect(screen, 0, 0, 0, 0);
}

template<typename T>
void Interpolate(const T & a, const T & b, vector<T> & result) {
	// If we have one point or less, we need special treatement
	if (1 >= result.size()) {
		// If we should interpolate no point, do nothing
		if (result.size() == 0) return;

		// Only one point is needed
		result[0] = a;
		return;
	}

	// Calculate how far every step will go
	T step = (b - a) * (1.0f / (result.size() - 1));

	// Setup the value to assign
	T value = a;

	// Loop through the result
	for (auto it = result.begin(); it != result.end(); ++it) {
		// Assign the value and increase it by the step
		*it = value;
		value += step;
	}
}
