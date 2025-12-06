#ifndef VOLUMETRIC_H
#define VOLUMETRIC_H

#include <Vector3f.h>
#include <cmath>
#include <vector>
#include <random>
#include "ray.hpp"
#include "hit.hpp"
#include "material.hpp"
#include "object3d.hpp"
#include "utils.hpp"
#include "Monte_Carlo.hpp"
// 体积介质类
class VolumetricMedium {
public:
    Vector3f scatteringCoeff;   // 散射系数 (sigma_s)
    Vector3f absorptionCoeff;   // 吸收系数 (sigma_a) 
    Vector3f emissionCoeff;     // 发射系数
    float phaseG;               // Henyey-Greenstein相位函数参数
    
    VolumetricMedium(const Vector3f& scatter = Vector3f(0.1f, 0.1f, 0.1f),
                     const Vector3f& absorb = Vector3f(0.05f, 0.05f, 0.05f),
                     const Vector3f& emission = Vector3f::ZERO,
                     float g = 0.0f) 
        : scatteringCoeff(scatter), absorptionCoeff(absorb), emissionCoeff(emission), phaseG(g) {}
    
    // 消光系数
    Vector3f extinctionCoeff() const {
        return scatteringCoeff + absorptionCoeff;
    }
    
    // 单次散射反照率
    Vector3f albedo() const {
        Vector3f ext = extinctionCoeff();
        return Vector3f(
            ext.x() > EPSILON ? scatteringCoeff.x() / ext.x() : 0.0f,
            ext.y() > EPSILON ? scatteringCoeff.y() / ext.y() : 0.0f,
            ext.z() > EPSILON ? scatteringCoeff.z() / ext.z() : 0.0f
        );
    }
    
    // Henyey-Greenstein相位函数
    float phaseFunction(const Vector3f& wi, const Vector3f& wo) const {
        float cosTheta = Vector3f::dot(wi.normalized(), wo.normalized());
        float denom = 1.0f + phaseG * phaseG + 2.0f * phaseG * cosTheta;
        return (1.0f - phaseG * phaseG) / (4.0f * M_PI * pow(denom, 1.5f));
    }
    
    // 采样散射方向
    Vector3f samplePhaseFunction(const Vector3f& wi) const {
        if (abs(phaseG) < EPSILON) {
            // 各向同性散射
            float phi = 2.0f * M_PI * u(mt);
            float cosTheta = 2.0f * u(mt) - 1.0f;
            float sinTheta = sqrt(1.0f - cosTheta * cosTheta);
            return Vector3f(sinTheta * cos(phi), sinTheta * sin(phi), cosTheta);
        } else {
            // Henyey-Greenstein采样
            float xi = u(mt);
            float cosTheta;
            if (abs(phaseG) > EPSILON) {
                float sqrTerm = (1.0f - phaseG * phaseG) / (1.0f + phaseG - 2.0f * phaseG * xi);
                cosTheta = (1.0f + phaseG * phaseG - sqrTerm * sqrTerm) / (2.0f * phaseG);
            } else {
                cosTheta = 2.0f * xi - 1.0f;
            }
            
            float sinTheta = sqrt(1.0f - cosTheta * cosTheta);
            float phi = 2.0f * M_PI * u(mt);
            
            // 构建局部坐标系
            Vector3f w = -wi.normalized();
            Vector3f u_vec = Vector3f::cross((abs(w.x()) > 0.1 ? Vector3f(0, 1, 0) : Vector3f(1, 0, 0)), w).normalized();
            Vector3f v_vec = Vector3f::cross(w, u_vec);
            
            return sinTheta * cos(phi) * u_vec + sinTheta * sin(phi) * v_vec + cosTheta * w;
        }
    }
};

// 体积渲染器
class VolumetricRenderer {
public:
    static Vector3f transmittance(const Ray& ray, float tMax, Object3D* scene, 
                                 const std::vector<VolumetricMedium*>& media) {
        if (media.empty()) return Vector3f(1.0f, 1.0f, 1.0f);
        
        Vector3f tau = Vector3f::ZERO;
        float stepSize = tMax / 32.0f; // 简化采样
        
        for (float t = 0; t < tMax; t += stepSize) {
            Vector3f pos = ray.pointAtParameter(t);
            for (auto* medium : media) {
                tau += medium->extinctionCoeff() * stepSize;
            }
        }
        
        return Vector3f(exp(-tau.x()), exp(-tau.y()), exp(-tau.z()));
    }
    
    static Vector3f renderPixel(const Ray& ray, Object3D* scene, 
                               const std::vector<Object3D*>& lights,
                               const std::vector<VolumetricMedium*>& media) {
        if (media.empty()) {
            // 如果没有体积介质，返回零（应该由外部处理表面渲染）
            return Vector3f::ZERO;
        }
        
        Vector3f L = Vector3f::ZERO;
        Vector3f throughput = Vector3f(1.0f, 1.0f, 1.0f);
        Ray currentRay = ray;
        
        for (int bounce = 0; bounce < MAX_DEPTH; ++bounce) {
            Hit hit;
            bool hasIntersection = scene->intersect(currentRay, hit, EPSILON);
            float tMax = hasIntersection ? hit.getT() : 1000.0f;
            
            // 体积采样
            Vector3f samplePos;
            VolumetricMedium* sampledMedium = nullptr;
            float sampleT;
            
            if (sampleVolume(currentRay, tMax, media, samplePos, sampledMedium, sampleT)) {
                // 在体积中发生散射
                L += throughput * sampledMedium->emissionCoeff;
                
                // 直接光照
                for (auto* light : lights) {
                    L += throughput * estimateDirectLighting(samplePos, sampledMedium, light, scene, media);
                }
                
                // 更新throughput并采样新方向
                Vector3f albedo = sampledMedium->albedo();
                throughput = Vector3f(throughput.x() * albedo.x(), 
                                    throughput.y() * albedo.y(), 
                                    throughput.z() * albedo.z());
                
                Vector3f newDir = sampledMedium->samplePhaseFunction(-currentRay.getDirection());
                currentRay = Ray(samplePos, newDir);
                
                // 俄罗斯轮盘
                if (bounce > 3) {
                    float q = std::max(0.05f, 1.0f - std::max({throughput.x(), throughput.y(), throughput.z()}));
                    if (u(mt) < q) break;
                    throughput = throughput / (1.0f - q);
                }
            } else {
                // 击中表面或逃逸
                if (hasIntersection) {
                    // 计算到表面的透射率
                    Vector3f trans = transmittance(currentRay, tMax, scene, media);
                    throughput = Vector3f(throughput.x() * trans.x(), 
                                        throughput.y() * trans.y(), 
                                        throughput.z() * trans.z());
                    
                    Material* mat = hit.getMaterial();
                    L += throughput * mat->getEmission();
                    
                    // 表面散射（简化处理）
                    Vector3f diffuse = mat->getDiffuseColor();
                    if (diffuse.length() > EPSILON) {
                        Vector3f normal = hit.getNormal().normalized();
                        Vector3f newDir = sampleHemisphere(normal);
                        currentRay = Ray(currentRay.pointAtParameter(tMax), newDir);
                        throughput = Vector3f(throughput.x() * diffuse.x(), 
                                            throughput.y() * diffuse.y(), 
                                            throughput.z() * diffuse.z());
                    } else {
                        break;
                    }
                } else {
                    // 逃逸到无穷远
                    break;
                }
            }
        }
        
        return L;
    }

private:
    static bool sampleVolume(const Ray& ray, float tMax, 
                           const std::vector<VolumetricMedium*>& media,
                           Vector3f& samplePos, VolumetricMedium*& sampledMedium, 
                           float& sampleT) {
        if (media.empty()) return false;
        
        // 简化：使用第一个介质并进行均匀采样
        sampledMedium = media[0];
        Vector3f sigma_t = sampledMedium->extinctionCoeff();
        float maxSigma = std::max({sigma_t.x(), sigma_t.y(), sigma_t.z()});
        
        if (maxSigma < EPSILON) return false;
        
        // Delta tracking / Woodcock tracking simplified
        float t = 0;
        while (t < tMax) {
            t += -log(u(mt)) / maxSigma;
            if (t >= tMax) return false;
            
            samplePos = ray.pointAtParameter(t);
            if (u(mt) < maxSigma / maxSigma) { // 简化版本
                sampleT = t;
                return true;
            }
        }
        
        return false;
    }
    
    static Vector3f estimateDirectLighting(const Vector3f& pos, VolumetricMedium* medium,
                                         Object3D* light, Object3D* scene,
                                         const std::vector<VolumetricMedium*>& media) {
        // 简化的直接光照估计
        Material* lightMat = light->getMaterial();
        Vector3f emission = lightMat->getEmission();
        
        if (emission.length() < EPSILON) return Vector3f::ZERO;
        
        // 简单地假设点光源在原点
        Vector3f lightPos = Vector3f::ZERO; // 应该从光源几何中获取
        Vector3f lightDir = (lightPos - pos).normalized();
        float lightDist = (lightPos - pos).length();
        
        // 阴影检测
        Ray shadowRay(pos, lightDir);
        Hit shadowHit;
        if (scene->intersect(shadowRay, shadowHit, EPSILON) && shadowHit.getT() < lightDist) {
            return Vector3f::ZERO;
        }
        
        // 体积透射率
        Vector3f trans = transmittance(shadowRay, lightDist, scene, media);
        
        // 相位函数
        float phase = medium->phaseFunction(-lightDir, lightDir); // 简化
        
        return emission * medium->scatteringCoeff * phase * trans / (lightDist * lightDist);
    }
    
    static Vector3f sampleHemisphere(const Vector3f& normal) {
        float xi1 = u(mt);
        float xi2 = u(mt);
        
        float cosTheta = xi1;
        float sinTheta = sqrt(1.0f - cosTheta * cosTheta);
        float phi = 2.0f * M_PI * xi2;
        
        Vector3f u_vec = Vector3f::cross((abs(normal.x()) > 0.1 ? Vector3f(0, 1, 0) : Vector3f(1, 0, 0)), normal).normalized();
        Vector3f v_vec = Vector3f::cross(normal, u_vec);
        
        return sinTheta * cos(phi) * u_vec + sinTheta * sin(phi) * v_vec + cosTheta * normal;
    }
};

// 体积介质预设
namespace VolumetricPresets {
    inline VolumetricMedium* createFog() {
        return new VolumetricMedium(
            Vector3f(0.1f, 0.1f, 0.1f),    // 散射
            Vector3f(0.05f, 0.05f, 0.05f),  // 吸收
            Vector3f::ZERO,                  // 无发射
            0.0f                            // 各向同性
        );
    }
    
    inline VolumetricMedium* createSmoke() {
        return new VolumetricMedium(
            Vector3f(0.3f, 0.3f, 0.3f),
            Vector3f(0.2f, 0.2f, 0.2f),
            Vector3f::ZERO,
            0.3f  // 轻微前向散射
        );
    }
    
    inline VolumetricMedium* createSubsurface() {
        return new VolumetricMedium(
            Vector3f(1.0f, 0.5f, 0.3f),    // 红色散射强
            Vector3f(0.1f, 0.3f, 0.5f),    // 蓝色吸收强
            Vector3f::ZERO,
            0.8f  // 强前向散射
        );
    }
}

#endif // VOLUMETRIC_H