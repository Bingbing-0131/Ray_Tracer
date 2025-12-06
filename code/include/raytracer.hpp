#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <iostream>

#include "scene_parser.hpp"
#include "ray.hpp"
#include "hit.hpp"
#include "group.hpp"
#include "light.hpp"
//参考smallpt

const int MAX_DEPTH_M = 5;  // 最大递归深度
const float MIN_T_M = 1e-3f; // 最小t值，避免自相交

    Vector3f getReflectionDirection(const Ray &ray, const Hit &hit) const {
        Vector3f n = hit.getNormal().normalized();         
        Vector3f v = ray.getDirection().normalized();   
        
        Vector3f r = (v - 2 * Vector3f::dot(n, v) * n).normalized();
        return r;
    }

    bool getRefractionDirection(const Ray &ray, const Hit &hit, float n1, float n2, Vector3f &refracted) const {
        Vector3f incident = ray.getDirection().normalized();
        Vector3f normal = hit.getNormal().normalized();
        
        bool fromInside = Vector3f::dot(incident, normal) > 0;
        if (fromInside) {
            normal = -normal;
        }
        
        float eta = n1 / n2;
        float cosI = -Vector3f::dot(normal, incident);
        float sinT2 = eta * eta * (1.0f - cosI * cosI);
        
        if (sinT2 >= 1.0f) {
            return false;
        }
        
        float cosT = std::sqrt(1.0f - sinT2);
        refracted = eta * incident + (eta * cosI - cosT) * normal;
        refracted = refracted.normalized();
        return true;
    }

    // 计算菲涅尔反射系数
    float computeFresnelReflectance(const Vector3f &incident, const Vector3f &normal, float n1, float n2) const {
        float cosI = -Vector3f::dot(normal, incident);
        float sinT2 = (n1 / n2) * (n1 / n2) * (1.0f - cosI * cosI);
        
        if (sinT2 >= 1.0f) {
            return 1.0f; // 全反射
        }
        
        float cosT = sqrt(1.0f - sinT2);
        float Rs = ((n1 * cosI - n2 * cosT) / (n1 * cosI + n2 * cosT));
        float Rp = ((n1 * cosT - n2 * cosI) / (n1 * cosT + n2 * cosI));
        
        return (Rs * Rs + Rp * Rp) / 2.0f;
    }

    // 生成cosine加权的半球方向（用于漫反射采样）
    Vector3f sampleCosineHemisphere(const Vector3f &normal, float r1, float r2) const {
        Vector3f w = normal;
        Vector3f u = ((abs(w.x()) > 0.1f) ? Vector3f(0, 1, 0) : Vector3f(1, 0, 0));
        u = Vector3f::cross(u, w).normalized();
        Vector3f v = Vector3f::cross(w, u);
        
        float cos_theta = sqrt(r1);
        float sin_theta = sqrt(1 - r1);
        float phi = 2 * M_PI * r2;
        
        return (u * cos(phi) * sin_theta + v * sin(phi) * sin_theta + w * cos_theta).normalized();
    }


// 递归的光线追踪函数 - 基于提供的伪代码算法
Vector3f traceRay(const Ray &ray, SceneParser &parser, int depth) {
    if (depth > MAX_DEPTH_M) {
        return Vector3f::ZERO;
    }
    
    Hit hit;
    if (!parser.getGroup()->intersect(ray, hit, MIN_T_M)) {
        // 未击中任何物体，返回背景色
        return parser.getBackgroundColor();
    }
    
    Material* material = hit.getMaterial();
    Vector3f hitPoint = ray.pointAtParameter(hit.getT());
    Vector3f normal = hit.getNormal().normalized();
    

    Vector3f color = Vector3f::ZERO;
    
    for (int i = 0; i < parser.getNumLights(); i++) {
        Light* light = parser.getLight(i);
        
        Vector3f lightDir, lightColor;
        light->getIllumination(hitPoint, lightDir, lightColor);
        
        // 阴影检测    perhaps if the object is outside the light, some problems will occur
        Ray shadowRay(hitPoint, lightDir);
        Hit shadowHit;
        bool inShadow = parser.getGroup()->intersect(shadowRay, shadowHit, MIN_T_M);
        
        if (!inShadow) {
            // 使用材质的diff_coef来调制局部光照
            Vector3f localShading = material->Shade(ray, hit, lightDir, lightColor);
            color += material->getDiffCoef() * localShading;
        }
    }
    
    
    if (material->getSpecCoef() > 0.0f) {
        Vector3f reflectDir = material->getReflectionDirection(ray, hit);
        Ray reflectRay(hitPoint, reflectDir);
        
        Vector3f reflectColor = traceRay(reflectRay, parser, depth + 1);
        color += material->getSpecCoef() * material->getReflectiveColor() * reflectColor;
    }
    
    
    if (material->getRefractivity() > 0.0f) {
        Vector3f incident = ray.getDirection();
        float n1 = 1.0f; // 假设光线从空气中来
        float n2 = material->getRefractiveIndex();
        
        // 判断是否从内部射出
        bool fromInside = Vector3f::dot(incident, normal) > 0;
        if (fromInside) {
            normal = -normal;
            std::swap(n1, n2);
        }
        
        Vector3f refractDir;
        if (material->getRefractionDirection(ray, hit, n1, n2, refractDir)) {
            // 发生折射
            Ray refractRay(hitPoint, refractDir);
            Vector3f refractColor = traceRay(refractRay, parser, depth + 1);
            color += material->getRefractivity() * material->getRefractiveColor() * refractColor;
        } else {
            // 全反射 - 当发生全反射时，折射能量转为反射
            Vector3f reflectDir = material->getReflectionDirection(ray, hit);
            Ray reflectRay(hitPoint, reflectDir);
            Vector3f reflectColor = traceRay(reflectRay, parser, depth + 1);
            color += material->getRefractivity() * material->getRefractiveColor() * reflectColor;
        }
    }
    
    return color;
}
