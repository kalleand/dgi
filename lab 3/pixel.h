#ifndef KALLAND_EFAHLEN_PIXEL
#define KALLAND_EFAHLEN_PIXEL

#include <glm/glm.hpp>

struct Pixel
{
    int x;
    int y;
    float zinv;
    glm::vec3 illumination;
};

#endif
