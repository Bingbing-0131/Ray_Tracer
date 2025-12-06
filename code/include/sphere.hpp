#ifndef SPHERE_H
#define SPHERE_H

#include "object3d.hpp"
#include "utils.hpp"
#include <vecmath.h>
#include <cmath>
//独立完成

// TODO: Implement functions and add more fields as necessary

class Sphere : public Object3D {
public:
    Sphere() {
        center = Vector3f(0, 0, 0);
        radius = 1.0f;
        area = 4*M_PI*radius*radius;
    }

    Sphere(const Vector3f &_center, float _radius, Material *material) : Object3D(material) {
        this->center = _center;
        this->radius = _radius;
        area = 4*M_PI*radius*radius;
    }
    

    ~Sphere() override = default;
    
    float getArea() override {
        return area;
    }
    
    void samplelight(Ray& ray, float& pdf, Material* &m) override {
        float theta = 2.0 * M_PI * u(mt), phi = M_PI * u(mt);
        Vector3f d(cosf(phi),sinf(phi)*cosf(theta),sinf(phi)*sinf(theta));
        ray = Ray(center+d*radius,d);
        m = material;
        pdf = 1.0/area;
    }

    bool intersect(const Ray &r, Hit &h, float tmin, float tmax) override {
        Vector3f oc = r.getOrigin() - center;

        float b = Vector3f::dot(oc, r.getDirection());
        float c = Vector3f::dot(oc, oc) - radius * radius;
        float delta = b * b - c;

        if (delta < 0) {
            return false;
        }
        
        delta = sqrt(delta);
        float t1 = -b - delta;
        float t2 = -b + delta;
        float t = t1;
        if (t < tmin) {
            t = t2;
        }
        if (t < tmin) {
            return false;
        }
        if (t < h.getT()) {
            Vector3f intersectionPoint = r.pointAtParameter(t);
            Vector3f normal = (intersectionPoint - center).normalized();
            h.set(t,material,normal,material->getDiffuseColor());
            return true;
        }

        return false;
    }

    bool getBoundingBox(BoundingBox& box) override {
        Vector3f min_point = center - Vector3f(radius, radius, radius);
        Vector3f max_point = center + Vector3f(radius, radius, radius);
        box = BoundingBox(min_point, max_point);
        return true;
    }

protected:
    Vector3f center;
    float radius;
    float area;
};

#endif