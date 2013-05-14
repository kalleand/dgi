#include <iostream>
#include <glm/glm.hpp>
#include <SDL.h>
#include "SDLauxiliary.h"
#include "TestModel.h"
#include <vector>

using glm::vec3;
using glm::vec2;
using glm::ivec2;
using glm::mat3;
using std::vector;
using std::numeric_limits;
using std::cout;
using std::endl;

// GLOBAL VARIABLES

const int SCREEN_WIDTH = 500;
const int SCREEN_HEIGHT = 500;
SDL_Surface* screen;
int t;
std::vector<Triangle> triangles;
vec3 camera_position(0, 0, -3.001);
vec3 current_color;
mat3 R;
const float VELOCITY = 0.001f;
float yaw = 0.0f;
float yawDelta = 0.0007371f;

// FUNCTIONS

void Interpolate(ivec2 a, ivec2 b, std::vector<ivec2> & result);
void VertexShader(const vec3 & v, ivec2 & p);
void DrawLineSDL(SDL_Surface * surface, ivec2 a, ivec2 b, vec3 color);
void DrawPolygonEdges(const vector<vec3> & vertices);
void ComputePolygonRows(const vector<ivec2> & vertexPixels,
                        vector<ivec2> & leftPixels,
                        vector<ivec2> & rightPixels);
void DrawPolygonRows(const vector<ivec2> & leftPixels,
                     const vector<ivec2> & rightPixels);
void DrawPolygon(const vector<vec3> & vertices);
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

    for (int i = 0; i < triangles.size(); ++i) {
    	current_color = triangles[i].color;

        std::vector<vec3> vertices(3);
        vertices[0] = triangles[i].v0;
        vertices[1] = triangles[i].v1;
        vertices[2] = triangles[i].v2;

        DrawPolygon(vertices);
    }

    if (SDL_MUSTLOCK(screen))
        SDL_UnlockSurface(screen);

    SDL_UpdateRect( screen, 0, 0, 0, 0 );
}

void Interpolate(ivec2 a, ivec2 b, std::vector<ivec2> & result) {
	int N = result.size();
	vec2 step = vec2(b - a) / float(glm::max(N - 1, 1));
	vec2 current(a);
	for (int i = 0; i < N; ++i) {
		result[i] = current;
		current += step;
	}
}

void VertexShader(const vec3 & v, ivec2 & p) {
    vec3 p_prim = (v - camera_position) * R;
    float f = 500.0f;
    printf("%f,%f,%f\n", v.x, v.y, v.z);
    p.x = f * p_prim.x / p_prim.z + SCREEN_WIDTH / 2;
    p.y = f * p_prim.y / p_prim.z + SCREEN_HEIGHT / 2;

    return;
}

void DrawLineSDL(SDL_Surface * surface, ivec2 a, ivec2 b, vec3 color) {
	ivec2 delta = glm::abs(a - b);
	int pixels = glm::max(delta.x, delta.y) + 1;
	vector<ivec2> line(pixels);
	Interpolate(a, b, line);

	for (auto it = line.begin(); it != line.end(); ++it) {
		PutPixelSDL(surface, (*it).x, (*it).y, color);
	}
}

void DrawPolygonEdges(const vector<vec3> & vertices) {
	int V = vertices.size();

	// Transform each vertex from 3D world position to 2D image position:
	vector<ivec2> projectedVertices(V);
	for (int i = 0; i < V; ++i) {
		VertexShader(vertices[i], projectedVertices[i]);
	}

	// Loop over all vertices and draw the edge from it to the next vertex:
	for (int i = 0; i < V; ++i) {
		int j = (i + 1) % V; // The next vertex
		vec3 color(1, 1, 1);
		DrawLineSDL(screen, projectedVertices[i], projectedVertices[j], color);
	}
}

void ComputePolygonRows(const vector<ivec2> & vertexPixels,
                        vector<ivec2> & leftPixels,
                        vector<ivec2> & rightPixels) {
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

		ivec2 delta = glm::abs(vertexPixels[i] - vertexPixels[j]);
		int pixels = glm::max(delta.x, delta.y) + 1;
		vector<ivec2> line(pixels);
		Interpolate(vertexPixels[i], vertexPixels[j], line);

		for (auto it = line.begin(); it != line.end(); ++it) {
			int k = (*it).y;
			leftPixels[k].x = glm::min(leftPixels[k].x, (*it).x);
			rightPixels[k].x = glm::max(rightPixels[k].x, (*it).x);
		}
	}
}

void DrawPolygonRows(const vector<ivec2> & leftPixels,
                     const vector<ivec2> & rightPixels) {
	for (int i = 0; i < leftPixels.size(); ++i) {
		int y = leftPixels[i].y;

		for (int x = leftPixels[i].x; x <= rightPixels[i].x; ++x) {
			PutPixelSDL(screen, x, y, current_color);
		}
	}
}

void DrawPolygon(const vector<vec3> & vertices) {
	vector<ivec2> vertexPixels(vertices.size());

	for (int i = 0; i < vertices.size(); ++i) {
		VertexShader(vertices[i], vertexPixels[i]);
	}

	vector<ivec2> leftPixels;
	vector<ivec2> rightPixels;
	ComputePolygonRows(vertexPixels, leftPixels, rightPixels);
	DrawPolygonRows(leftPixels, rightPixels);
}

void UpdateR() {
	R = mat3(cos(yaw), 0, -sin(yaw),               
	                0, 1,         0,
	         sin(yaw), 0,  cos(yaw));
}
