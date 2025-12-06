#ifndef PLANE_H
#define PLANE_H

#include "object3d.hpp"
#include "utils.hpp"
#include <vecmath.h>
#include <cmath>
//独立完成
class Plane : public Object3D {
public:
    Plane():n(Vector3f::UP) ,D(0.0f){
        
    }

    Plane(const Vector3f &normal, float d, Material *m)
        : Object3D(m), n(normal.normalized()), D(d) {}

    ~Plane() override = default;

    bool intersect(const Ray &r, Hit &h, float tmin, float tmax) override {
        Vector3f R_o = r.getOrigin();
        Vector3f R_d = r.getDirection();
    
        float denom = Vector3f::dot(n, R_d);
        if (fabs(denom) < 1e-6) return false;
    
        float t = (D + Vector3f::dot(n, R_o)) / denom;
        if (t < tmin || t >= h.getT()) return false;
    
        h.set(t, material, n);
    
            return true;
    } 

    bool getBoundingBox(BoundingBox& box) override {
        return false;
    }

protected:
    Vector3f n;
    float D;
};

#endif // PLANE_H
