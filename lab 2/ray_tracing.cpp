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

constexpr double pi() { return atan(1) / 4; } // Define pi as arctan(1) / 4

// ----------------------------------------------------------------------------
// GLOBAL VARIABLES

const int SCREEN_WIDTH = 1280;
const int SCREEN_HEIGHT = 720;
const float VELOCITY = 0.001f; // Velocity of the camera every tick.
const float EPSILON = 0.00001f; // Rounding error.

// The screen to draw on.
SDL_Surface* screen;
// The triangles that make the scene.
std::vector<Triangle> triangles;
// Number of ticks since last update.
int ti;
// Focal length, determines the field of view.
float focal_length = SCREEN_HEIGHT / 1.5f;
// Original camera position.
vec3 camera_position(0, 0.25f, -1.5);

// Global illumination color.
vec3 indirectLight = vec3( 0.2f, 0.2f, 0.2f );
// The change of angle when camera is moved around.
float yawDelta = 0.0007371f;
// The angle that the camera should rotate.
float yaw = 0.0f;
// The rotation matrix.
mat3 R;

// Original light position.
vec3 lightPos(0, -0.5, -0.7);
// Vector of original light color.
vec3 lightColor = 1.f * vec3(1, 1, 1);

/**
 * Declares the intersection with the position, distance and what triangle
 * the intersection was at.
 */
struct Intersection {
    vec3 position;
    float distance;
    int triangleIndex;
};

// ----------------------------------------------------------------------------
// FUNCTIONS


void Update();
void Draw();
void updateR();
bool ClosestIntersection(
        const vec3& start,
        const vec3& dir,
        const std::vector<Triangle>& triangles,
        Intersection& closestIntersection);
vec3 DirectLight(const Intersection& i);

/**
 * Main function. Initializes everything and contain the main loop which calls
 * update and then draw.
 */
int main(int argc, char* argv[]) {
    screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT);
    ti = SDL_GetTicks(); // Set start value for timer.
    LoadTestModel(triangles); // Load the model with triangles.
    updateR(); // Initialize R matrix.

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

    // Find out pressed keys.
    Uint8* keystate = SDL_GetKeyState( 0);

    // Determine what direction forward is.
    vec3 forward(R[2][0], R[2][1], R[2][2]);

    // Forwards.
    if (keystate[SDLK_UP]) {
        camera_position += forward * VELOCITY * dt;
    }

    // Backwards.
    if (keystate[SDLK_DOWN]) {
        camera_position -= forward * VELOCITY * dt;
    }

    // Turn left.
    if (keystate[SDLK_LEFT]) {
        yaw -= yawDelta * dt;
        updateR();
    }

    // Turn right.
    if (keystate[SDLK_RIGHT]) {
        yaw += yawDelta * dt;
        updateR();
    }

    // Move light source forward.
    if (keystate[SDLK_w]) {
        lightPos.z += VELOCITY * dt;
    }

    // Move light source backward.
    if (keystate[SDLK_s]) {
        lightPos.z -= VELOCITY * dt;
    }

    // Move light source to the left.
    if (keystate[SDLK_a]) {
        lightPos.x -= VELOCITY * dt;
    }

    // Move light source to the right.
    if (keystate[SDLK_d]) {
        lightPos.x += VELOCITY * dt;
    }

    // Move light source down.
    if (keystate[SDLK_q]) {
        lightPos.y -= VELOCITY * dt;
    }

    // Move light source up.
    if (keystate[SDLK_e]) {
        lightPos.y += VELOCITY * dt;
    }
}

/**
 * Draws the scene as described by the triangles, light source and
 * cameraposition.
 *
 * Uses ray tracing to draw the scene.
 */
void Draw() {
    if (SDL_MUSTLOCK(screen))
        SDL_LockSurface(screen);

// We parallelize for speedup.
#pragma omp parallel for
    for (int y = 0; y < SCREEN_HEIGHT; ++y) {
        Intersection intersection;

        for (int x = 0; x < SCREEN_WIDTH; ++x) {

            // Determine direction of the ray.
            vec3 dir(x - SCREEN_WIDTH / 2, y - SCREEN_HEIGHT / 2, focal_length);
            dir = R * dir;

            // Default color is black.
            vec3 color(0, 0, 0);
            // Calculate closest intersection.
            if (ClosestIntersection(camera_position, dir, triangles, intersection)) {
                // Receive the color of this triangle and mix it with the global
                // illumination.
                color = triangles[intersection.triangleIndex].color;
                color *= (DirectLight(intersection) + indirectLight);
            }

            // Paint the pixel in its color.
            PutPixelSDL(screen, x, y, color);
        }
    }

    if (SDL_MUSTLOCK(screen))
        SDL_UnlockSurface(screen);

    SDL_UpdateRect(screen, 0, 0, 0, 0);
}

/**
 * Updates the rotation matrix R to rotate the yaw degrees.
 */
void updateR() {
    R = mat3(cos(yaw), 0, -sin(yaw),
                    0, 1,         0,
             sin(yaw), 0,  cos(yaw));
}

/**
 * Determines the color of the Intersection depending on if it is subject to
 * direct light of the light source and the angle of these light particles.
 */
vec3 DirectLight(const Intersection & i) {
    // Get distance from light source to intersection.
    vec3 distance = lightPos - i.position;
    // Get the backwards direction that the light travels.
    vec3 r = glm::normalize(distance);
    // The euclidean distance to the light source from the intersection.
    float radius = glm::length(distance);

    Intersection ci;
    // Get the direction that the light travels.
    vec3 dir = glm::normalize(i.position - lightPos);

    // If it the intersection point is obscured from the light source by another
    // triangle we want to return black.
    if (ClosestIntersection(lightPos, dir, triangles, ci)) {
        if (ci.distance < radius - EPSILON)
            return vec3(0, 0, 0);
    }

    // Get the normal of the triangle of the intersection.
    vec3 n = triangles[i.triangleIndex].normal;

    // Calculate the dot product between the normal and the direction of the
    // light.
    float scalar = glm::dot(r, n);

    // Return the intensity of the color of the light source.
    float divisor = 4 * pi() * radius * radius;
    return lightColor * std::max(scalar, 0.0f) * (1.0f / divisor);
}

/**
 * Returns the closest intersection.
 */
bool ClosestIntersection(
        const vec3& start,
        const vec3& dir,
        const std::vector<Triangle>& triangles,
        Intersection & closestIntersection) {
    // Default min distance to a triangle. This returns the maximum value of a
    // float.
    float current_min = std::numeric_limits<float>::max();
    // Default is to not have found an intersection.
    bool found_triangle = false;

    // define x vector as (t, u, v) which is what we want.
    vec3 x;
    float & t = x.x;
    float & u = x.y;
    float & v = x.z;

    // Iterate over every triangle to find the closest intersection of the ray.
    for (int i = 0; i < triangles.size(); ++i) {
        // Direction of camera position to one edge in the triangle.
        vec3 b = start - triangles[i].v0;

        // Construct the A matrix used to calculate the intersection.
        mat3 A(-dir, triangles[i].e1, triangles[i].e2);
        // x is received by calculating the inverse of A and then performing a
        // matrix multiplication with b.
        x = glm::inverse(A) * b;

        // Is this inside the triangle?
        if(u >= 0 && v >= 0 && (u+v) <= 1 && t >= 0) {
            // Get the coordinate of the intersection.
            vec3 coord = triangles[i].v0 + (triangles[i].e1 * u) + (triangles[i].e2 * v);
            // Get the euclidean distance to this coordinate.
            float length = glm::distance(coord, start);
            // Check if this intersection is the closest yet.
            if(length < current_min) {
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
