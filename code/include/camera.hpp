#ifndef CAMERA_H
#define CAMERA_H

#include "ray.hpp"
#include <vecmath.h>
#include <float.h>
#include <cmath>


class Camera {
public:
    Camera(const Vector3f &center, const Vector3f &direction, const Vector3f &up, int imgW, int imgH) {
        this->center = center;
        this->direction = direction.normalized();
        this->horizontal = Vector3f::cross(this->direction, up).normalized();
        this->up = Vector3f::cross(this->horizontal, this->direction);
        this->width = imgW;
        this->height = imgH;
    }

    // Generate rays for each screen-space coordinate
    virtual Ray generateRay(const Vector2f &point) = 0;
    virtual ~Camera() = default;

    int getWidth() const { return width; }
    int getHeight() const { return height; }

protected:
    // Extrinsic parameters
    Vector3f center;
    Vector3f direction;
    Vector3f up;
    Vector3f horizontal;
    // Intrinsic parameters
    int width;
    int height;
};

// TODO: Implement Perspective camera
// You can add new functions or variables whenever needed.
class PerspectiveCamera : public Camera {

    public:
        PerspectiveCamera(const Vector3f &center, const Vector3f &direction,
                const Vector3f &up, int imgW, int imgH, float angle) : Camera(center, direction, up, imgW, imgH) {
    

            float halfHeight = tan(angle / 2.0);
            float halfWidth = halfHeight * ((float)imgW / (float)imgH);
            
            fy = (height / 2.0) / halfHeight;
            fx = (width / 2.0) / halfWidth;
            

            cx = width / 2.0;
            cy = height / 2.0;
        }
    
        Ray generateRay(const Vector2f &point) override {
            Vector3f directionCamera = Vector3f(
                (point.x() - cx) / fx,
                (cy - point.y()) / fy,
                1.0f
            ).normalized();
        
            Vector3f worldDirection = 
                horizontal * directionCamera.x() -
                up * directionCamera.y() +
                direction * directionCamera.z();
        
            return Ray(center, worldDirection.normalized());
        }
        
    
    private:
        float fx, fy; // Scale parameters from image space to world space
        float cx, cy; // Optical center (usually the image center)
    };

#endif //CAMERA_H
