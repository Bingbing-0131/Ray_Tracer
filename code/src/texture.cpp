#include "texture.hpp"
#include "stb_image.h"
#include <cstring>

#define STB_IMAGE_IMPLEMENTATION
//stbi的使用来源于claude

// Helper to clamp coordinates within image bounds
Vector3f Texture::image_color(int u, int v) {
    if (x < 0) x = 0;
    else if (x >= w) x = w - 1;

    if (y < 0) y = 0;
    else if (y >= h) y = h - 1;

    const int idx = (y * w + x) * channel;
    return Vector3f(
        fig[idx],
        fig[idx + 1],
        fig[idx + 2]
    ) / 255.f;
}

//bilinear interpolation
Vector3f Texture::get_color(float u, float v) {
    u = u - floorf(u);
    v = v - floorf(v);

    if (u < 0) u += 1.f;
    if (v < 0) v += 1.f;

    const float fx = u * w;
    const float fy = (1.f - v) * h;

    const int x = static_cast<int>(fx);
    const int y = static_cast<int>(fy);

    const float dx = fx - x;
    const float dy = fy - y;

    const Vector3f c00 = image_color(x,     y);
    const Vector3f c10 = image_color(x + 1, y);
    const Vector3f c01 = image_color(x,     y + 1);
    const Vector3f c11 = image_color(x + 1, y + 1);

    const Vector3f c0 = c00 * (1 - dx) + c10 * dx;
    const Vector3f c1 = c01 * (1 - dx) + c11 * dx;

    return c0 * (1 - dy) + c1 * dy;
}

// Texture constructor: load image and set texture state
Texture::Texture(const char* filename) {
    if (!filename || filename[0] == '\0') {
        is_texture = false;
        return;
    }

    file = filename;
    fig = stbi_load(filename, &w, &h, &channel, 0);
    is_texture = fig != nullptr;
}
