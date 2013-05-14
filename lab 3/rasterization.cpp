#include <iostream>
#include <glm/glm.hpp>
#include <SDL.h>
#include "SDLauxiliary.h"
#include "TestModel.h"
#include "../lab 1/interpolate.h"

using glm::vec3;
using glm::ivec2;
using glm::mat3;

// GLOBAL VARIABLES

const int SCREEN_WIDTH = 500;
const int SCREEN_HEIGHT = 500;
SDL_Surface* screen;
int t;
std::vector<Triangle> triangles;
vec3 camera_pos(0, 0, -3.001);
mat3 R;
float yaw = 0.0f;

// FUNCTIONS

void VertexShader(const vec3 & v, ivec2 & p);
void Update();
void Draw();
void UpdateR();

int main(int argc, char* argv[]) {
    LoadTestModel( triangles );
    screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT );
    t = SDL_GetTicks();	// Set start value for timer.

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
    float dt = float(t2-t);
    t = t2;
    std::cout << "Render time: " << dt << " ms." << std::endl;

    Uint8* keystate = SDL_GetKeyState(0);

    if (keystate[SDLK_UP])
        ;

    if (keystate[SDLK_DOWN])
        ;

    if (keystate[SDLK_RIGHT])
        ;

    if (keystate[SDLK_LEFT])
        ;

    if (keystate[SDLK_RSHIFT])
        ;

    if (keystate[SDLK_RCTRL])
        ;

    if (keystate[SDLK_w])
        ;

    if (keystate[SDLK_s])
        ;

    if (keystate[SDLK_d])
        ;

    if (keystate[SDLK_a])
        ;

    if (keystate[SDLK_e])
        ;

    if (keystate[SDLK_q])
        ;
}

void Draw()
{
    SDL_FillRect(screen, 0, 0);
    UpdateR();

    if (SDL_MUSTLOCK(screen))
        SDL_LockSurface(screen);

    for (int i = 0; i < triangles.size(); ++i)
    {
        std::vector<vec3> vertices(3);

        vertices[0] = triangles[i].v0;
        vertices[1] = triangles[i].v1;
        vertices[2] = triangles[i].v2;

        // Add drawing
        for (int j = 0; j < 3; ++j) {
            ivec2 proj_pos;
            VertexShader(vertices[j], proj_pos);
            vec3 color(1, 1, 1);
            PutPixelSDL(screen, proj_pos.x, proj_pos.y, color);
        }
    }

    if (SDL_MUSTLOCK(screen))
        SDL_UnlockSurface(screen);

    SDL_UpdateRect( screen, 0, 0, 0, 0 );
}

void VertexShader(const vec3 & v, ivec2 & p)
{
    vec3 p_prim = (v - camera_pos) * R;
    float f = 500.0f;
    printf("%f,%f,%f\n", v.x, v.y, v.z);
    p.x = f * p_prim.x / p_prim.z + SCREEN_WIDTH / 2;
    p.y = f * p_prim.y / p_prim.z + SCREEN_HEIGHT / 2;

    return;
}

void UpdateR()
{
    R = mat3(cos(yaw), 0, -sin(yaw),
            0, 1, 0,
        sin(yaw), 0, cos(yaw));
}
