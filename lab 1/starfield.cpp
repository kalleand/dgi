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
#include "timer.h"
#include <unistd.h>
#include <chrono>
#include <thread>
#include <math.h>
#include <algorithm>

using std :: this_thread :: sleep_for;
using std :: chrono :: nanoseconds;
using std :: vector;
using glm :: vec2;
using glm :: vec3;

// --------------------------------------------------------
// GLOBAL VARIABLES

const float MAX_FPS = 30.0f;
const int SCREEN_WIDTH = 640;
const int SCREEN_HEIGHT = 480;
SDL_Surface * screen;
vector<vec3> stars(1000), previousStars(1000);
int t;

// --------------------------------------------------------
// FUNCTION DECLARATIONS

float random_float();
void drawLine(float fromU, float fromV, float toU, float toV, vec3 color);

void Initialize();
void Update();
void Draw();

// --------------------------------------------------------
// FUNCTION DEFINITIONS

int main(int argc, char * argv[]) {
	Initialize();

	while (NoQuitMessageSDL()) {
		double before = read_timer();
		Update();
		Draw();

		double time = read_timer() - before;
		double wait = 1.0f / MAX_FPS - time;

		if (wait > 0)
			sleep_for(nanoseconds((int) round(wait * 1000000000)));
	}

	SDL_SaveBMP(screen, "screenshot.bmp");
	return EXIT_SUCCESS;
}


float random_float() {
	return float(rand()) / float(RAND_MAX);
}

void Initialize() {
	screen = InitializeSDL(SCREEN_WIDTH, SCREEN_HEIGHT);

	for (int i = 0; i < stars.size(); ++i) {
		stars[i].x = -1 + 2 * random_float();
		stars[i].y = -1 + 2 * random_float();
		stars[i].z = random_float();

		previousStars[i] = stars[i];
	}

	t = SDL_GetTicks();
}

void Update() {
	int t2 = SDL_GetTicks();
	float dt = float(t2 - t);
	t = t2;

	vec3 velocity(0, 0, -0.0005f);

	for (int i = 0; i < stars.size(); ++i) {
		previousStars[i] = stars[i];
		stars[i] += dt * velocity;

		if (stars[i].z <= 0) {
			stars[i].z += 1;
			stars[i].x = -1 + 2 * random_float();
			stars[i].y = -1 + 2 * random_float();

			previousStars[i] = stars[i];
		}
	}
}

void drawLine(float fromU, float fromV, float toU, float toV, vec3 color) {
	vec2 start(fromU, fromV);
	vec2 delta(toU - fromU, toV - fromV);

	if (fromU < 0) {
		if (delta.x == 0.0f) return;

		fromV = start.y - start.x * (delta.y / delta.x);
		fromU = 0.0f;
		return drawLine(fromU, fromV, toU, toV, color);
	}

	if (fromU > SCREEN_WIDTH) {
		if (delta.x == 0.0f) return;

		fromV = start.y + (SCREEN_WIDTH - start.x) * (delta.y / delta.x);
		fromU = SCREEN_WIDTH;
		return drawLine(fromU, fromV, toU, toV, color);
	}

	if (toU < 0) {
		if (delta.x == 0.0f) return;

		toV = start.y - start.x * (delta.y / delta.x);
		toU = 0.0f;
		return drawLine(fromU, fromV, toU, toV, color);
	}

	if (toU > SCREEN_WIDTH) {
		if (delta.x == 0.0f) return;

		toV = start.y + (SCREEN_WIDTH - start.x) * (delta.y / delta.x);
		toU = SCREEN_WIDTH;
		return drawLine(fromU, fromV, toU, toV, color);
	}

	if (fromV < 0) {
		if (delta.y == 0.0f) return;

		fromU = start.x - start.y * (delta.x / delta.y);
		fromV = 0.0f;
		return drawLine(fromU, fromV, toU, toV, color);
	}

	if (fromV > SCREEN_HEIGHT) {
		if (delta.y == 0.0f) return;

		fromU = start.x + (SCREEN_HEIGHT - start.y) * (delta.x / delta.y);
		fromV = SCREEN_HEIGHT;
		return drawLine(fromU, fromV, toU, toV, color);
	}

	if (toV < 0) {
		if (delta.y == 0.0f) return;

		toU = start.x - start.y * (delta.x / delta.y);
		toV = 0.0f;
		return drawLine(fromU, fromV, toU, toV, color);
	}

	if (toV > SCREEN_HEIGHT) {
		if (delta.y == 0.0f) return;

		toU = start.x + (SCREEN_HEIGHT - start.y) * (delta.x / delta.y);
		toV = SCREEN_HEIGHT;
		return drawLine(fromU, fromV, toU, toV, color);
	}

	vec2 step = glm :: normalize(delta);
	int steps = ceil(glm :: length(delta));

	for (vec2 v = start; steps > 0; v += step, --steps)
		PutPixelSDL(screen, v.x, v.y, color);
}

void Draw() {
	float f = SCREEN_HEIGHT / 4;

	SDL_FillRect(screen, 0, 0);
	if (SDL_MUSTLOCK(screen))
		SDL_LockSurface(screen);

	for (int i = 0; i < stars.size(); ++i) {
		float fromU = f * previousStars[i].x / previousStars[i].z + SCREEN_WIDTH / 2;
		float fromV = f * previousStars[i].y / previousStars[i].z + SCREEN_HEIGHT / 2;
		float toU = f * stars[i].x / stars[i].z + SCREEN_WIDTH / 2;
		float toV = f * stars[i].y / stars[i].z + SCREEN_HEIGHT / 2;

		vec3 color = vec3(0.2f) / (stars[i].z * stars[i].z);

		drawLine(fromU, fromV, toU, toV, color);
	}

	if (SDL_MUSTLOCK(screen))
		SDL_UnlockSurface(screen);

	SDL_UpdateRect(screen, 0, 0, 0, 0);
}
