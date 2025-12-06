#ifndef FOG_SPHERE_H
#define FOG_SPHERE_H

#include <Vector3f.h>
#include <cmath>
#include "ray.hpp"
#include "utils.hpp"

// 简单的雾球类
class FogSphere {
public:
    Vector3f center;
    float radius;
    float density;          // 雾的密度
    Vector3f scatterColor;  // 散射颜色
    float absorption;       // 吸收系数
    
    FogSphere(const Vector3f& c, float r, float d = 0.1f, 
              const Vector3f& color = Vector3f(0.8f, 0.9f, 1.0f), 
              float abs = 0.05f)
        : center(c), radius(r), density(d), scatterColor(color), absorption(abs) {}
    
    // 检查射线是否与雾球相交
    bool intersect(const Ray& ray, float& tNear, float& tFar) const {
        Vector3f oc = ray.getOrigin() - center;
        float a = Vector3f::dot(ray.getDirection(), ray.getDirection());
        float b = 2.0f * Vector3f::dot(oc, ray.getDirection());
        float c = Vector3f::dot(oc, oc) - radius * radius;
        float discriminant = b * b - 4 * a * c;
        
        if (discriminant < 0) return false;
        
        float sqrt_discriminant = sqrt(discriminant);
        tNear = (-b - sqrt_discriminant) / (2.0f * a);
        tFar = (-b + sqrt_discriminant) / (2.0f * a);
        
        // 确保tNear和tFar是正数
        if (tFar < 0) return false;
        if (tNear < 0) tNear = 0;
        
        return true;
    }
    
    // 计算透射率（Beer定律）
    float transmittance(float distance) const {
        return exp(-density * absorption * distance);
    }
    
    // 计算雾的散射贡献
    Vector3f scatteringContribution(const Ray& ray, float tNear, float tFar, 
                                   const Vector3f& lightDir, const Vector3f& lightColor) const {
        if (tFar <= tNear) return Vector3f::ZERO;
        
        Vector3f scattering = Vector3f::ZERO;
        float stepSize = (tFar - tNear) / 16.0f; // 16步采样
        
        for (int i = 0; i < 16; i++) {
            float t = tNear + (i + 0.5f) * stepSize;
            Vector3f samplePos = ray.pointAtParameter(t);
            
            // 计算到此点的透射率
            float transmission = transmittance(t - tNear);
            
            // 简单的相位函数（各向同性散射）
            float phase = 1.0f / (4.0f * M_PI);
            
            // 前向散射增强
            float cosTheta = Vector3f::dot(-ray.getDirection().normalized(), lightDir.normalized());
            float forwardScatter = 1.0f + 0.5f * cosTheta;
            
            // 散射贡献
            Vector3f localScatter = lightColor * scatterColor * density * 
                                   transmission * phase * forwardScatter * stepSize;
            
            scattering += localScatter;
        }
        
        return scattering;
    }
};

// 全局雾球列表
static std::vector<FogSphere> g_fogSpheres;

// 添加雾球
void addFogSphere(const FogSphere& fog) {
    g_fogSpheres.push_back(fog);
}

// 清除所有雾球
void clearFogSpheres() {
    g_fogSpheres.clear();
}

// 计算射线穿过所有雾球的效果
Vector3f calculateFogEffect(const Ray& ray, float maxDistance, 
                           const Vector3f& originalColor,
                           const Vector3f& lightDir, const Vector3f& lightColor) {
    if (g_fogSpheres.empty()) return originalColor;
    
    Vector3f finalColor = originalColor;
    float totalTransmission = 1.0f;
    Vector3f totalScattering = Vector3f::ZERO;
    
    for (const auto& fog : g_fogSpheres) {
        float tNear, tFar;
        if (fog.intersect(ray, tNear, tFar)) {
            // 限制在maxDistance内
            tFar = std::min(tFar, maxDistance);
            if (tNear >= maxDistance) continue;
            
            // 计算这个雾球的透射率
            float fogDistance = tFar - tNear;
            float fogTransmission = fog.transmittance(fogDistance);
            totalTransmission *= fogTransmission;
            
            // 计算散射贡献
            Vector3f scattering = fog.scatteringContribution(ray, tNear, tFar, lightDir, lightColor);
            totalScattering += scattering * totalTransmission;
        }
    }
    
    return finalColor * totalTransmission + totalScattering;
}

// 便利函数：创建常见的雾效果
namespace FogPresets {
    FogSphere createBasicFog(const Vector3f& center, float radius) {
        return FogSphere(center, radius, 0.1f, Vector3f(0.8f, 0.9f, 1.0f), 0.05f);
    }
    
    FogSphere createDenseFog(const Vector3f& center, float radius) {
        return FogSphere(center, radius, 0.3f, Vector3f(0.7f, 0.8f, 0.9f), 0.15f);
    }
    
    FogSphere createColoredSmoke(const Vector3f& center, float radius, const Vector3f& color) {
        return FogSphere(center, radius, 0.2f, color, 0.1f);
    }
    
    FogSphere createSubtleAtmosphere(const Vector3f& center, float radius) {
        return FogSphere(center, radius, 0.05f, Vector3f(0.9f, 0.95f, 1.0f), 0.02f);
    }
}

#endif // FOG_SPHERE_H