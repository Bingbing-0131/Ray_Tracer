#ifndef BOUNDINGBOX_H
#define BOUNDINGBOX_H

#include "utils.hpp"
#include "ray.hpp"
#include <algorithm>
#include <string>
#include <sstream>
//独立完成
class BoundingBox {
private:
    Vector3f min_point, max_point;
    bool is_empty;

public:
    BoundingBox() : is_empty(true) {}
    
    BoundingBox(const Vector3f& min_p, const Vector3f& max_p) 
        : min_point(min_p), max_point(max_p), is_empty(false) {}
    
    // Extend bounding box to include point
    void extend(const Vector3f& point) {
        if (is_empty) {
            min_point = max_point = point;
            is_empty = false;
        } else {
            for (int i = 0; i < 3; i++) {
                min_point[i] = std::min(min_point[i], point[i]);
                max_point[i] = std::max(max_point[i], point[i]);
            }
        }
    }
    
    // Union of two bounding boxes
    static BoundingBox Union(const BoundingBox& b1, const BoundingBox& b2) {
        if (b1.is_empty) return b2;
        if (b2.is_empty) return b1;
        
        BoundingBox result;
        result.is_empty = false;
        for (int i = 0; i < 3; i++) {
            result.min_point[i] = std::min(b1.min_point[i], b2.min_point[i]);
            result.max_point[i] = std::max(b1.max_point[i], b2.max_point[i]);
        }
        return result;
    }
    
    // Ray-box intersection test (simple version)
    bool intersect(const Ray& ray, float t_min, float t_max) const {
        if (is_empty) return false;
        
        Vector3f origin = ray.getOrigin();
        Vector3f direction = ray.getDirection();
        
        for (int i = 0; i < 3; i++) {
            float inv_dir = 1.0f / direction[i];
            float t0 = (min_point[i] - origin[i]) * inv_dir;
            float t1 = (max_point[i] - origin[i]) * inv_dir;
            
            if (inv_dir < 0.0f) {
                std::swap(t0, t1);
            }
            
            t_min = std::max(t0, t_min);
            t_max = std::min(t1, t_max);
            
            if (t_max < t_min) {
                return false;
            }
        }
        
        return true;
    }
    
    // Ray-box intersection test with near/far distances
    bool intersect(const Ray& ray, float t_min, float t_max, float& t_near, float& t_far) const {
        if (is_empty) {
            t_near = t_far = 0;
            return false;
        }
        
        Vector3f origin = ray.getOrigin();
        Vector3f direction = ray.getDirection();
        
        t_near = t_min;
        t_far = t_max;
        
        for (int i = 0; i < 3; i++) {
            if (std::abs(direction[i]) < 1e-8f) {
                // Ray is parallel to the slab
                if (origin[i] < min_point[i] || origin[i] > max_point[i]) {
                    return false;
                }
            } else {
                float inv_dir = 1.0f / direction[i];
                float t0 = (min_point[i] - origin[i]) * inv_dir;
                float t1 = (max_point[i] - origin[i]) * inv_dir;
                
                if (inv_dir < 0.0f) {
                    std::swap(t0, t1);
                }
                
                t_near = std::max(t0, t_near);
                t_far = std::min(t1, t_far);
                
                if (t_near > t_far) {
                    return false;
                }
            }
        }
        
        return t_far > 0; // Only return true if intersection is in front of ray
    }
    
    // Check if two bounding boxes intersect
    static bool Intersects(const BoundingBox& b1, const BoundingBox& b2) {
        if (b1.is_empty || b2.is_empty) return false;
        
        for (int i = 0; i < 3; i++) {
            if (b1.max_point[i] < b2.min_point[i] || b1.min_point[i] > b2.max_point[i]) {
                return false;
            }
        }
        return true;
    }
    
    // Get center point
    Vector3f getCenter() const {
        if (is_empty) return Vector3f(0, 0, 0);
        return (min_point + max_point) * 0.5f;
    }
    
    // Get surface area
    float getSurfaceArea() const {
        if (is_empty) return 0.0f;
        
        Vector3f extent = max_point - min_point;
        return 2.0f * (extent[0] * extent[1] + 
                      extent[1] * extent[2] + 
                      extent[2] * extent[0]);
    }
    
    // Get volume
    float getVolume() const {
        if (is_empty) return 0.0f;
        
        Vector3f extent = max_point - min_point;
        return extent[0] * extent[1] * extent[2];
    }
    
    // Getters
    Vector3f getMin() const { return min_point; }
    Vector3f getMax() const { return max_point; }
    bool isEmpty() const { return is_empty; }
    
    // String representation for debugging
    std::string toString() const {
        if (is_empty) return "Empty BoundingBox";
        
        std::stringstream ss;
        ss << "BoundingBox[(" << min_point[0] << "," << min_point[1] << "," << min_point[2] 
           << ") to (" << max_point[0] << "," << max_point[1] << "," << max_point[2] << ")]";
        return ss.str();
    }
};

#endif // BOUNDINGBOX_H