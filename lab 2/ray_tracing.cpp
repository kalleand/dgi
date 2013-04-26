#include <iostream>
#include <glm/glm.hpp>
#include <SDL.h>
#include "SDLauxiliary.h"
#include "TestModel.h"
#include <omp.h>
#include <cmath>
#include <algorithm>

using glm::vec3;
using glm::mat3;

constexpr double pi() { return atan(1) / 4; }

// ----------------------------------------------------------------------------
// GLOBAL VARIABLES

const int SCREEN_WIDTH = 1280;
const int SCREEN_HEIGHT = 720;
const float VELOCITY = 0.001f;
const float EPSILON = 0.00001f;

SDL_Surface* screen;
std::vector<Triangle> triangles;
int ti;
float focal_length = SCREEN_HEIGHT / 1.5f;
vec3 camera_position(0, 0.25f, -1.5);

vec3 indirectLight = vec3( 0.2f, 0.2f, 0.2f );
float yawDelta = 0.0007371f;
float yaw = 0.0f;
mat3 R;

vec3 lightPos(0, -0.5, -0.7);
vec3 lightColor = 1.f * vec3(1, 1, 1);

// ----------------------------------------------------------------------------
// FUNCTIONS

struct Intersection {
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
vec3 DirectLight(const Intersection& i);

void updateR();

int main(int argc, char* argv[]) {
    screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT);
    ti = SDL_GetTicks(); // Set start value for timer.
    LoadTestModel(triangles);
    updateR();

    while (NoQuitMessageSDL()) {
        Update();
        Draw();
    }

    SDL_SaveBMP(screen, "screenshot.bmp");
    return 0;
}

void Update() {
    // Compute frame time:
    int t2 = SDL_GetTicks();
    float dt = float(t2-ti);
    ti = t2;
    std::cout << "Render time: " << dt << " ms." << std::endl;
    Uint8* keystate = SDL_GetKeyState( 0);

    vec3 forward(R[2][0], R[2][1], R[2][2]);

    if (keystate[SDLK_UP]) {
        camera_position += forward * VELOCITY * dt;
    }

    if (keystate[SDLK_DOWN]) {
        camera_position -= forward * VELOCITY * dt;
    }

    if (keystate[SDLK_LEFT]) {
        yaw -= yawDelta * dt;
        updateR();
    }

    if (keystate[SDLK_RIGHT]) {
        yaw += yawDelta * dt;
        updateR();
    }
    if (keystate[SDLK_w]) {
        lightPos.z += VELOCITY * dt;
    }
    if (keystate[SDLK_s]) {
        lightPos.z -= VELOCITY * dt;
    }
    if (keystate[SDLK_a]) {
        lightPos.x -= VELOCITY * dt;
    }
    if (keystate[SDLK_d]) {
        lightPos.x += VELOCITY * dt;
    }
    if (keystate[SDLK_q]) {
        lightPos.y -= VELOCITY * dt;
    }
    if (keystate[SDLK_e]) {
        lightPos.y += VELOCITY * dt;
    }
}

void Draw() {
    if (SDL_MUSTLOCK(screen))
        SDL_LockSurface(screen);

#pragma omp parallel for
    for (int y = 0; y < SCREEN_HEIGHT; ++y) {
        Intersection intersection;

        for (int x = 0; x < SCREEN_WIDTH; ++x) {
            vec3 dir(x - SCREEN_WIDTH / 2, y - SCREEN_HEIGHT / 2, focal_length);
            dir = R * dir;

            vec3 color(0, 0, 0);
            if (ClosestIntersection(camera_position, dir, triangles, intersection)) {
                color = triangles[intersection.triangleIndex].color;
                color *= (DirectLight(intersection) + indirectLight);
            }

            PutPixelSDL(screen, x, y, color);
        }
    }

    if (SDL_MUSTLOCK(screen))
        SDL_UnlockSurface(screen);

    SDL_UpdateRect(screen, 0, 0, 0, 0);
}

void updateR() {
    R = mat3(cos(yaw), 0, -sin(yaw),
                    0, 1,         0,
             sin(yaw), 0,  cos(yaw));
}

vec3 DirectLight(const Intersection & i) {
    vec3 distance = lightPos - i.position;
    vec3 r = glm::normalize(distance);
    float radius = glm::length(distance);

    Intersection ci;
    vec3 dir = glm::normalize(i.position - lightPos);

    if (ClosestIntersection(lightPos, dir, triangles, ci)) {
        if (ci.distance < radius - EPSILON)
            return vec3(0, 0, 0);
    }

    vec3 n = triangles[i.triangleIndex].normal;

    float scalar = glm::dot(r, n);
    float divisor = 4 * pi() * radius * radius;
    return lightColor * std::max(scalar, 0.0f) * (1.0f / divisor);
}

bool ClosestIntersection(
        const vec3& start,
        const vec3& dir,
        const std::vector<Triangle>& triangles,
        Intersection & closestIntersection) {
    float current_min = std::numeric_limits<float>::max();
    bool found_triangle = false;

    vec3 x;
    float & t = x.x;
    float & u = x.y;
    float & v = x.z;

    for (int i = 0; i < triangles.size(); ++i) {
        vec3 b = start - triangles[i].v0;

        mat3 A(-dir, triangles[i].e1, triangles[i].e2);
        x = glm::inverse(A) * b;

        // Is this inside the triangle?
        if(u >= 0 && v >= 0 && (u+v) <= 1 && t >= 0)
        {
            vec3 coord = triangles[i].v0 + (triangles[i].e1 * u) + (triangles[i].e2 * v);
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
