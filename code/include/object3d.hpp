#ifndef OBJECT3D_H
#define OBJECT3D_H

#include "ray.hpp"
#include "hit.hpp"
#include "material.hpp"
#include "bounding_box.hpp"  // Fixed include name to match your file
//独立完成

// Base class for all 3d entities.
class Object3D {
public:
    Object3D() : material(nullptr) {}
    
    virtual ~Object3D() = default;
    
    explicit Object3D(Material *material) {
        this->material = material;
    }
    
    Material* getMaterial() const { return material; }
    
    virtual float getArea() { return 0; }
    virtual void samplelight(Ray& ray, float& pdf, Material* &m) {}
    
    // Intersect Ray with this object. If hit, store information in hit structure.
    virtual bool intersect(const Ray &r, Hit &h, float tmin, float tmax = MAX_EPS) { return false; }
    
    // Pure virtual function that must be implemented by all derived classes
    virtual bool getBoundingBox(BoundingBox &box) = 0;

protected:
    Material *material;
};

#endif