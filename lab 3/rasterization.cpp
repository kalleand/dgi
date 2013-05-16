#include <iostream>
#include <glm/glm.hpp>
#include <SDL.h>
#include "SDLauxiliary.h"
#include "TestModel.h"
#include <vector>
#include "pixel.h"
#include "vertex.h"
#include <cmath>

using glm::vec3;
using glm::vec2;
using glm::ivec2;
using glm::mat3;
using std::vector;
using std::numeric_limits;
using std::cout;
using std::endl;

// GLOBAL VARIABLES

constexpr double pi() { return atan(1) / 4; } // Define pi as arctan(1) / 4

const int SCREEN_WIDTH = 500;
const int SCREEN_HEIGHT = 500;
SDL_Surface* screen;
int t;
std::vector<Triangle> triangles;
vec3 camera_position(0, 0, -3.001);

vec3 currentNormal;
vec3 currentReflectance;

mat3 R;
const float VELOCITY = 0.001f;
float yaw = 0.0f;
float yawDelta = 0.0007371f;
float depthBuffer[SCREEN_HEIGHT][SCREEN_WIDTH];

vec3 lightPos(0, -0.5, -0.7);
vec3 lightPower = 1.1f * vec3(1, 1, 1);
vec3 indirectLightPowerPerArea = 0.5f *vec3(1, 1, 1);

// FUNCTIONS

void Interpolate(Pixel a, Pixel b, std::vector<Pixel> & result);
void VertexShader(const Vertex & v, Pixel & p);
void PixelShader(const Pixel & p);
void ComputePolygonRows(const vector<Pixel> & vertexPixels,
                        vector<Pixel> & leftPixels,
                        vector<Pixel> & rightPixels);
void DrawPolygonRows(const vector<Pixel> & leftPixels,
                     const vector<Pixel> & rightPixels);
void DrawPolygon(const vector<Vertex> & vertices);
void Update();
void Draw();
void UpdateR();

int main(int argc, char* argv[]) {
    LoadTestModel( triangles );
    screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT );
    t = SDL_GetTicks(); // Set start value for timer.

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

    // Find out pressed keys.
    Uint8* keystate = SDL_GetKeyState(0);

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
        UpdateR();
    }

    // Turn right.
    if (keystate[SDLK_RIGHT]) {
        yaw += yawDelta * dt;
        UpdateR();
    }

    if (keystate[SDLK_RSHIFT])
        ;

    if (keystate[SDLK_RCTRL])
        ;

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

void Draw() {
    SDL_FillRect(screen, 0, 0);
    UpdateR();

    // Clear depth buffer
    for (int y = 0; y < SCREEN_HEIGHT; y++) {
        for (int x = 0; x < SCREEN_WIDTH; x++) {
            depthBuffer[y][x] = 0;
        }
    }


    if (SDL_MUSTLOCK(screen))
        SDL_LockSurface(screen);

    for (int i = 0; i < triangles.size(); ++i) {
        std::vector<Vertex> vertices(3);
        vertices[0].position = triangles[i].v0;
        vertices[1].position = triangles[i].v1;
        vertices[2].position = triangles[i].v2;

        currentNormal = triangles[i].normal;
        currentReflectance = triangles[i].color;

        DrawPolygon(vertices);
    }

    if (SDL_MUSTLOCK(screen))
        SDL_UnlockSurface(screen);

    SDL_UpdateRect( screen, 0, 0, 0, 0 );
}

// We need a special function to interpolate pixels, since we need to use
// floating point number as coordinate steps.
void Interpolate(Pixel a, Pixel b, std::vector<Pixel> & result) {
    int N = result.size();
    float divisor = float(glm::max(N - 1, 1));

    float dx = (b.x - a.x) / divisor;
    float dy = (b.y - a.y) / divisor;
    float dzinv = (b.zinv - a.zinv) / divisor;
    vec3 dpos3d = (b.pos3d - a.pos3d) * (1.0f /  divisor);

    float xc = a.x;
    float yc = a.y;
    float zinvc = a.zinv;
    vec3 pos3dc = a.pos3d;

    for (int i = 0; i < N; i++) {
        result[i].x = round(xc);
        result[i].y = round(yc);
        result[i].zinv = zinvc;
        result[i].pos3d = pos3dc;

        xc += dx;
        yc += dy;
        zinvc += dzinv;
        pos3dc += dpos3d;
    }
}

void VertexShader(const Vertex & v, Pixel & p) {
    vec3 p_prim = (v.position - camera_position) * R;
    float f = SCREEN_WIDTH;
    p.x = round(f * p_prim.x / p_prim.z + SCREEN_WIDTH / 2);
    p.y = round(f * p_prim.y / p_prim.z + SCREEN_HEIGHT / 2);
    p.zinv = 1 / p_prim.z;
    p.pos3d = v.position;
}

void PixelShader(const Pixel & p) {
    int x = p.x;
    int y = p.y;

    if (x < 0 || x >= SCREEN_WIDTH ||
        y < 0 || y >= SCREEN_HEIGHT)
        return;

    if (p.zinv > depthBuffer[y][x]) {
        depthBuffer[y][x] = p.zinv;

        // Get distance from light source to intersection.
        vec3 distance = lightPos - p.pos3d;
        // Get the backwards direction that the light travels.
        vec3 r = glm::normalize(distance);
        // The euclidean distance to the light source from the intersection.
        float radius = glm::length(distance);

        float scalar = glm::dot(r, currentNormal);
        float divisor = 4 * pi() * radius * radius;
        vec3 D = lightPower * std::max(scalar, 0.0f) * (1.0f / divisor);
        vec3 illumination = currentReflectance * (D + indirectLightPowerPerArea);

        // Finally, draw the pixel
        PutPixelSDL(screen, x, y, illumination);
    }
}

void ComputePolygonRows(const vector<Pixel> & vertexPixels,
                        vector<Pixel> & leftPixels,
                        vector<Pixel> & rightPixels) {
    // 1. Find max and min y-value of the polygon
    // and compute the number of rows it occupies.
    int min_y = numeric_limits<int>::max();
    int max_y = numeric_limits<int>::min();

    for (auto it = vertexPixels.begin(); it != vertexPixels.end(); ++it) {
        min_y = glm::min(min_y, (*it).y);
        max_y = glm::max(max_y, (*it).y);
    }

    // 2. Resize leftPixels and rightPixels
    // so that they have an element for each row.
    leftPixels.resize(SCREEN_HEIGHT);
    rightPixels.resize(SCREEN_HEIGHT);

    // 3. Initialize the x-coordinates in leftPixels
    // to some really large value and the x-coordinates
    // in rightPixels to some really small value.
    for (int i = 0; i < leftPixels.size(); ++i) {
        leftPixels[i].x = SCREEN_WIDTH;
        leftPixels[i].y = i;

        rightPixels[i].x = 0;
        rightPixels[i].y = i;
    }

    // 4. Loop through all edges of the polygon and use
    // linear interpolation to find the x-coordinate for
    // each row it occupies. Update the corresponding
    // values in rightPixels and leftPixels.
    for (int i = 0; i < vertexPixels.size(); ++i) {
        int j = (i + 1) % vertexPixels.size();

        int deltax = glm::abs(vertexPixels[i].x - vertexPixels[j].x);
        int deltay = glm::abs(vertexPixels[i].y - vertexPixels[j].y);
        int pixels = glm::max(deltax, deltay) + 1;
        vector<Pixel> line(pixels);
        Interpolate(vertexPixels[i], vertexPixels[j], line);

        for (auto it = line.begin(); it != line.end(); ++it) {
            int k = (*it).y;
            if(k >= SCREEN_HEIGHT || k < 0) continue;
            if((*it).x < leftPixels[k].x) {
                leftPixels[k].x = (*it).x;
                leftPixels[k].zinv = (*it).zinv;
                leftPixels[k].pos3d = (*it).pos3d;
            }
            if((*it).x > rightPixels[k].x) {
                rightPixels[k].x = (*it).x;
                rightPixels[k].zinv = (*it).zinv;
                rightPixels[k].pos3d = (*it).pos3d;
            }
        }
    }
}

void DrawPolygonRows(const vector<Pixel> & leftPixels,
                     const vector<Pixel> & rightPixels) {
    for (int i = 0; i < leftPixels.size(); ++i) {
        int N = 1 + rightPixels[i].x - leftPixels[i].x;
        if (N <= 0) continue;
        vector<Pixel> row(N);
        Interpolate(leftPixels[i], rightPixels[i], row);

        for (auto it = row.begin(); it != row.end(); ++it) {
            PixelShader(*it);
        }
    }
}

void DrawPolygon(const vector<Vertex> & vertices) {
    vector<Pixel> vertexPixels(vertices.size());

    for (int i = 0; i < vertices.size(); ++i) {
        VertexShader(vertices[i], vertexPixels[i]);
    }

    vector<Pixel> leftPixels;
    vector<Pixel> rightPixels;
    ComputePolygonRows(vertexPixels, leftPixels, rightPixels);
    DrawPolygonRows(leftPixels, rightPixels);
}

void UpdateR() {
    R = mat3(cos(yaw), 0, -sin(yaw),               
                    0, 1,         0,
             sin(yaw), 0,  cos(yaw));
}
