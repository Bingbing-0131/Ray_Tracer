#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <iostream>
#include<ctime>
#include "utils.hpp"
#include "scene_parser.hpp"
#include "image.hpp"
#include "camera.hpp"
#include "group.hpp"
#include "light.hpp"
#include "Monte_Carlo.hpp"
#include "BVH.hpp"
#include <string>
#include <vector>


using namespace std;

int main(int argc, char *argv[]) {
    
    for (int argNum = 1; argNum < argc; ++argNum) {
        std::cout << "Argument " << argNum << " is: " << argv[argNum] << std::endl;
    }
    
    if (argc != 4) {
        cout << "Usage: ./bin/PA1 <input scene file> <output bmp file> <samples per pixel>" << endl;
        return 1;
    }
    
    string inputFile = argv[1];
    string outputFile = argv[2];  // only bmp is allowed.
    string spp = argv[3];
    int num_spp = atoi(spp.c_str());
    
    SceneParser sceneParser(inputFile.c_str());
    Image image(sceneParser.getCamera()->getWidth(), sceneParser.getCamera()->getHeight());
    
    // Get reference to the group and build KDTree from it
    Group* my_group = sceneParser.getGroup();
    KDTree kdtree(*my_group);  // Build KDTree from the group
    
    // Print KDTree statistics
    kdtree.printStats();
    std::cout<<"start"<<std::endl;
    
    #pragma omp parallel for schedule(dynamic, 1)
    int w = sceneParser.getCamera()->getWidth();
    int h = sceneParser.getCamera()->getWidth();
    for (int x = 0; x < w; x++) {
        fprintf(stderr, "\rPending: %.2f%%", (float)x / (float)sceneParser.getCamera()->getWidth() * 100.0);
        for (int y = 0; y < h; y++) {
            Vector3f color = Vector3f::ZERO;
            
            for (int i = 0; i < num_spp; i++) {
                Ray generate_ray = sceneParser.getCamera()->generateRay(Vector2f(x + u(mt), y + u(mt)));
                Vector3f t = pathtrace_nee(generate_ray, 0, &sceneParser, &kdtree,true);  // Pass pointer to kdtree
                color += clamp(t, 0.0f, 2.0f);
            }
            color = color /num_spp;
            image.SetPixel(x, y, color);
        }
    }
    
    fprintf(stderr, "\rRendering complete!                    \n");
    image.SaveBMP(outputFile.c_str());
    return 0;
}