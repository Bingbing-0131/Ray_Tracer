#define STB_IMAGE_IMPLEMENTATION
#ifndef TEXTURE_H
#define TEXTURE_H
#include <vecmath.h>
#include <string>
//独立完成

class Texture {
public:
    bool is_texture;
    std::string file;
    Texture(const char *file_);

    Texture(){
        is_texture = false;
    }
        std::string Getfile() const {
        return file;
    }


    Vector3f get_color(float u, float v);
    Vector3f fig_color(int u, int v);
private:
    int chaneel;
    int w;
    int h;
    unsigned char *fig;
};




#endif