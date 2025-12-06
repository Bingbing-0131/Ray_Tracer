#ifndef GROUP_HPP
#define GROUP_HPP

#include <vector>
#include <iostream>

#include "object3d.hpp"
#include "ray.hpp"
#include "hit.hpp"
#include "utils.hpp"
#include "bounding_box.hpp"
//独立完成

class Group : public Object3D {
public:
    Group() = default;

    explicit Group(int object_count) {
        objects.resize(object_count, nullptr);
    }

    ~Group() override {
        for (auto* obj : objects) {
            delete obj;
        }
        objects.clear();
    }

    bool intersect(const Ray& ray, Hit& hit, float t_min, float t_max = MAX_EPS) override {
        bool hit_detected = false;

        for (auto* obj : objects) {
            if (obj && obj->intersect(ray, hit, t_min)) {
                hit_detected = true;
            }
        }

        return hit_detected;
    }

    // 实现面光源采样
    void samplelight(Ray& ray, float& pdf, Material*& m) override {
        float area = 0;
        std::vector<Object3D*> emissive_objects;
        
        // 收集所有发光物体
        for(auto obj : objects) {
            if(obj->getMaterial() && obj->getMaterial()->getEmission().length() > EPSILON) {
                emissive_objects.push_back(obj);
                area += obj->getArea();
            }
        }

        if (emissive_objects.empty()) {
            pdf = 0.0f;
            m = nullptr;
            return;
        }
        
        // 按面积进行重要性采样
        float accumulated_area = 0;
        float p = u(mt) * area;
        

        for(auto obj : emissive_objects) {
            accumulated_area += obj->getArea();
            if(p <= accumulated_area) {
                obj->samplelight(ray, pdf, m);
                break;
            }
        }
    }

    bool getBoundingBox(BoundingBox& bbox) override {
        bool found_box = false;
        BoundingBox temp_box;

        for (auto* obj : objects) {
            if (!obj) continue;

            BoundingBox current;
            if (obj->getBoundingBox(current)) {
                if (!found_box) {
                    bbox = current;
                    found_box = true;
                } else {
                    bbox = BoundingBox::Union(bbox, current);
                }
            }
        }

        return found_box;
    }

    void addObject(int index, Object3D* obj) {
        if (index >= static_cast<int>(objects.size())) {
            objects.resize(index + 1, nullptr);
        }

        objects[index] = obj;
    }

    int getGroupSize() const {
        return static_cast<int>(objects.size());
    }

    std::vector<Object3D*> getObjects() const {
        return objects;
    }

private:
    std::vector<Object3D*> objects;
};

#endif  // GROUP_HPP
