#include <iostream>
#include <glm/glm.hpp>
#include <SDL.h>
#include "SDLauxiliary.h"
#include "TestModel.h"

using glm::vec3;
using glm::mat3;

// ----------------------------------------------------------------------------
// GLOBAL VARIABLES

const int SCREEN_WIDTH = 500;
const int SCREEN_HEIGHT = 500;
SDL_Surface* screen;
std::vector<Triangle> triangles;
int ti;
float focal_length = SCREEN_HEIGHT / 2;
vec3 camera_position(0, 0, -2);


// ----------------------------------------------------------------------------
// FUNCTIONS

struct Intersection
{
    vec3 position;
    float distance;
    int triangleIndex;
};

void Update();
void Draw();
bool ClosestIntersection(
        const vec3& start,
        const vec3& dir,
        const std::vector<Triangle>& triangles,
        Intersection& closestIntersection);


int main( int argc, char* argv[] )
{
    screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT );
    ti = SDL_GetTicks(); // Set start value for timer.
    LoadTestModel(triangles);

    while( NoQuitMessageSDL() )
    {
        Update();
        Draw();
    }

    SDL_SaveBMP( screen, "screenshot.bmp" );
    return 0;
}

void Update()
{
    // Compute frame time:
    int t2 = SDL_GetTicks();
    float dt = float(t2-ti);
    ti = t2;
    std::cout << "Render time: " << dt << " ms." << std::endl;
    Uint8* keystate = SDL_GetKeyState( 0 );
    if( keystate[SDLK_UP] )
    {
        camera_position.z += 0.1f;
    }
    if( keystate[SDLK_DOWN] )
    {
        camera_position.z -= 0.1f;
    }
    if( keystate[SDLK_LEFT] )
    {
        camera_position.x -= 0.1f;
    }
    if( keystate[SDLK_RIGHT] )
    {
        camera_position.x += 0.1f;
    }
}

void Draw() {
    if (SDL_MUSTLOCK(screen))
        SDL_LockSurface(screen);

    Intersection intersection;

    for (int y = 0; y < SCREEN_HEIGHT; ++y) {
		vec3 dir(0, y - SCREEN_HEIGHT / 2, focal_length);

        for (int x = 0; x < SCREEN_WIDTH; ++x) {
            dir.x = x - SCREEN_WIDTH / 2;

            vec3 color(0, 0, 0);
            if (ClosestIntersection(camera_position, dir, triangles, intersection)) {
                color = triangles[intersection.triangleIndex].color;
            }

            PutPixelSDL(screen, x, y, color);
        }
    }

    if (SDL_MUSTLOCK(screen))
        SDL_UnlockSurface(screen);

    SDL_UpdateRect(screen, 0, 0, 0, 0);
}

bool ClosestIntersection(
        const vec3& start,
        const vec3& dir,
        const std::vector<Triangle>& triangles,
        Intersection & closestIntersection)
{
    float current_min = std::numeric_limits<float>::max();
    bool found_triangle = false;
    for (int i = 0; i < triangles.size(); ++i)
    {
        vec3 e1 = triangles[i].v1 - triangles[i].v0;
        vec3 e2 = triangles[i].v2 - triangles[i].v0;
        vec3 b = start - triangles[i].v0;
        mat3 A(-dir, e1, e2);
        vec3 x = glm::inverse(A) * b;

        // Is this inside the triangle.
        float t = x.x;
        float u = x.y;
        float v = x.z;
        if(u >= 0 && v >= 0 && (u+v) <= 1 && t >= 0)
        {
            vec3 coord = triangles[i].v0 + (e1*u) + (e2*v);
            float length = glm::distance(coord, start);;
            if(length < current_min)
            {
                current_min = length;
                closestIntersection.position = coord;
                closestIntersection.distance = length;
                closestIntersection.triangleIndex = i;
                found_triangle = true;
            }
        }
    }
    return found_triangle;
}
