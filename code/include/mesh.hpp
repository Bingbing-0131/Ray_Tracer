#ifndef MESH_H
#define MESH_H

#include <vector>
#include "object3d.hpp"
#include "triangle.hpp"
#include "Vector2f.h"
#include "Vector3f.h"
#include "bounding_box.hpp"
#include "utils.hpp"
//独立完成
// Forward declaration
class KDTree;  // or class BVH; if you're using BVH instead

class Mesh : public Object3D {

public:
    Mesh(const char *filename, Material *m);
    ~Mesh();

    struct TriangleIndex {
        TriangleIndex() {
            x[0] = 0; x[1] = 0; x[2] = 0;
        }
        int &operator[](const int i) { return x[i]; }
        // By Computer Graphics convention, counterclockwise winding is front face
        int x[3]{};
    };

    std::vector<Vector3f> v;        // vertices
    std::vector<TriangleIndex> t;   // triangle indices
    std::vector<Vector3f> n;        // normals
    std::vector<Vector2f> uv;       // texture coordinates
    std::vector<Triangle*> triangles; // actual triangle objects

    bool intersect(const Ray &r, Hit &h, float tmin, float tmax=MAX_EPS) override;
    float getArea() override;
    void samplelight(Ray& r, float& pdf, Material*& m) override;
    bool getBoundingBox(BoundingBox& box) override;

private:
    KDTree* bvh_tree;  // Acceleration structure for fast intersection
    // or BVH* bvh_tree; if using BVH class
    
    // Normal can be used for light estimation
    void computeNormal();
};

#endif