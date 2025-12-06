#ifndef TRACE_H
#define TRACE_H
#pragma once
#include <iostream>
#include <Vector3f.h>
#include <cmath>
#include "utils.hpp"
#include "ray.hpp"
#include "object3d.hpp"
#include "hit.hpp"
#include "scene_parser.hpp"
#include "light.hpp"
#include "BVH.hpp"
#include "fog_shpere.hpp"  // 添加雾球支持
//这段代码参考了smallpt和GAMES101，测试场景同时也参考了这两份资料
float clamp(float x, float min, float max){
    return std::max(min, std::min(x, max));
}

Vector3f clamp(Vector3f x, float min, float max){
    return Vector3f(clamp(x.x(), min, max), clamp(x.y(), min, max), clamp(x.z(), min, max));
}

float fresnel(const Vector3f &I, const Vector3f &N, const float &ior){
     float cosi = clamp(-1, 1, Vector3f::dot(I, N));
    float etai = 1, etat = ior;
    if (cosi > 0) {  std::swap(etai, etat); }
    float sint = etai / etat * sqrtf(std::max(0.f, 1 - cosi * cosi));
    if (sint >= 1) {
        return 1.0f;
    }
    else {
        float cost = sqrtf(std::max(0.f, 1 - sint * sint));
        cosi = fabsf(cosi);
        float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
        float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
        return (Rs * Rs + Rp * Rp) / 2.0f;
    }
}

float geometrySchlickGGX(float cosTheta, float roughness) {
    float k = (roughness + 1.0f) * (roughness + 1.0f) / 8.0f;
    return cosTheta / (cosTheta * (1.0f - k) + k);
}

float compute_g(const Vector3f& wi, const Vector3f& wo, const Vector3f& n, float r) {
    float NdotWi = std::max(0.0f, Vector3f::dot(wi, n));
    float NdotWo = std::max(0.0f, Vector3f::dot(wo, n));
    float G1i = geometrySchlickGGX(NdotWi, r);
    float G1o = geometrySchlickGGX(NdotWo, r);
    return G1i * G1o;
}


float compute_d(const Vector3f& h, const Vector3f& n, float r){
    float cost = Vector3f::dot(h, n);
    float d = M_PI*pow(1.0 + cost * cost * (r*r - 1), 2);
    if (d < EPSILON)return 1.0f;
    else return r*r/d;
}

void brdf(const Vector3f& wi, const Vector3f& wo, const Vector3f& n, Material* m, Vector3f& color){
    if(m->useck){
        float f,g,d;
        Vector3f h = (-wo+wi).normalized();
        f = fresnel(wi, n, m->getRefractiveIndex());
        g = compute_g(wi, -wo, n, m->roughness);
        d = compute_d(h, n, m->roughness);
        Vector3f diff = m->getDiffuseColor()*(1.0-f)/M_PI;
        float div = 4*Vector3f::dot(wi, n)*Vector3f::dot(-wo, n);
        if(div<EPSILON)color =  (diff+Vector3f(1.0f))/P;
        else color = (diff+Vector3f(f*g*d/div))/P;
    }
    else{
        if(Vector3f::dot(wo, n)>0){
            color = color/M_PI;
        }
        else color = Vector3f::ZERO;
    }
}

Vector3f getReflectionDirection(const Ray &ray, const Hit &hit) {
        Vector3f n = hit.getNormal().normalized();         
        Vector3f v = ray.getDirection().normalized();   
        
        Vector3f r = (v - 2 * Vector3f::dot(n, v) * n).normalized();
        return r;
}

Vector3f sampleCosineHemisphere(const Vector3f &normal, float r1, float r2){
        Vector3f w = normal;
        Vector3f u = ((abs(w.x()) > 0.1f) ? Vector3f(0, 1, 0) : Vector3f(1, 0, 0));
        u = Vector3f::cross(u, w).normalized();
        Vector3f v = Vector3f::cross(w, u);
        
        float cos_theta = sqrt(r1);
        float sin_theta = sqrt(1 - r1);
        float phi = 2 * M_PI * r2;
        
        return (u * cos(phi) * sin_theta + v * sin(phi) * sin_theta + w * cos_theta).normalized();
}

bool getRefractionDirection(const Ray &ray, const Hit &hit, float n1, float n2, Vector3f &refracted)  {
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

Vector3f pathtrace_nee(const Ray& ray, int depth, SceneParser* sceneParser, KDTree* kdtree, bool fog, bool flag = true) {
    Vector3f L_direct = Vector3f::ZERO;
    Vector3f L_indirect = Vector3f::ZERO;
    
    Hit hit;
    if (!kdtree->intersect(ray, hit, EPSILON)) {
        return sceneParser->getBackgroundColor();
    }

    Vector3f hitPoint = ray.pointAtParameter(hit.getT());
    Vector3f normal = hit.getNormal().normalized();
    Vector3f shadingNormal = Vector3f::dot(normal, ray.getDirection()) < 0 ? normal : -normal;

    Material* material = hit.getMaterial();
    Vector3f baseColor = hit.t_color;

    // 自发光项
    L_direct += material->getEmission();

    if (depth > 5) {
        if (u(mt) < P) {
            baseColor = baseColor / P;
        } else {
            return L_direct;
        }
    }

    float randVal = u(mt);

    if (material->getDiffCoef() > 0.9f) {
        // 发光体自身不计算间接光
        if (material->getEmission() != Vector3f::ZERO) {
            return flag ? material->getEmission() : Vector3f::ZERO;
        }

        // NEE 光源采样部分
        float pdf_light = 1.0f;
        Ray lightSample;
        Material* lightMaterial = nullptr;

        kdtree->samplelight(lightSample, pdf_light, lightMaterial);

        Vector3f lightPos = lightSample.getOrigin();
        Vector3f toLight = (lightPos - hitPoint).normalized();
        Vector3f lightNormal = lightSample.getDirection().normalized();
        Vector3f lightIntensity = lightMaterial->getEmission();
        float lightDist = (lightPos - hitPoint).length();

        Ray shadowRay(hitPoint, toLight);
        Hit shadowHit;
        kdtree->intersect(shadowRay, shadowHit, EPSILON);
        float distToOcc = shadowHit.getT();

        if (lightDist - distToOcc < EPSILON) {
            float cosTheta = Vector3f::dot(toLight, normal);
            float cosPhi = Vector3f::dot(lightNormal, -toLight);
            Vector3f f_r = baseColor;
            brdf(ray.getDirection(), toLight, normal, material, f_r);

            float weight = 1.0f / (lightDist * lightDist * fmax(pdf_light, EPSILON));
            L_direct += lightIntensity * f_r * cosTheta * cosPhi * weight;
        }

        // 路径追踪部分
        float r1 = u(mt);
        float r2 = u(mt);
        Vector3f sampleDir = sampleCosineHemisphere(shadingNormal, r1, r2).normalized();
        Vector3f brdfVal = baseColor;
        brdf(sampleDir, ray.getDirection(), normal, material, brdfVal);

        if (brdfVal.length() < EPSILON) {
            brdfVal = baseColor;
        }

        L_indirect = brdfVal * pathtrace_nee(Ray(hitPoint, sampleDir), depth + 1, sceneParser, kdtree, false, false);
        return L_direct + L_indirect;
    }
    // 处理镜面反射
    else if (material->getSpecCoef() >0.0f) {
        Vector3f reflectDir = getReflectionDirection(ray, hit);
        Ray reflectRay(hitPoint, reflectDir);
        L_indirect = baseColor * pathtrace_nee(reflectRay, depth + 1, sceneParser, kdtree, false);
        return L_direct + L_indirect;
    }
    // 折射处理
    else {
        Vector3f refrDir;
        bool refracted = getRefractionDirection(ray, hit, 1.0f, material->getRefractiveIndex(), refrDir);
        if (refracted) {
            L_indirect = baseColor * pathtrace_nee(Ray(hitPoint, refrDir), depth + 1, sceneParser, kdtree, false, false);
        } else {
            L_indirect = Vector3f::ZERO;
        }
        return L_direct + L_indirect;
    }
}


Vector3f pathtrace_no_nee(const Ray& ray, int depth, SceneParser* sceneParser, KDTree* kdtree, bool fog, bool flag = true) {
    Vector3f L_direct = Vector3f::ZERO;
    Vector3f L_indirect = Vector3f::ZERO;
    
    Hit hit;
    if (!kdtree->intersect(ray, hit, EPSILON)) {
        return sceneParser->getBackgroundColor();
    }

    Vector3f hitPoint = ray.pointAtParameter(hit.getT());
    Vector3f normal = hit.getNormal().normalized();
    Vector3f shadingNormal = Vector3f::dot(normal, ray.getDirection()) < 0 ? normal : -normal;

    Material* material = hit.getMaterial();
    Vector3f baseColor = hit.t_color;

    // 自发光项
    L_direct += material->getEmission();

    if (depth > 5) {
        if (u(mt) < P) {
            baseColor = baseColor / P;
        } else {
            return L_direct;
        }
    }

    float randVal = u(mt);

    if (material->getDiffCoef() > 0.9f) {
        // 路径追踪部分
        float r1 = u(mt);
        float r2 = u(mt);
        Vector3f sampleDir = sampleCosineHemisphere(shadingNormal, r1, r2).normalized();
        Vector3f brdfVal = baseColor;
        brdf(sampleDir, ray.getDirection(), normal, material, brdfVal);

        if (brdfVal.length() < EPSILON) {
            brdfVal = baseColor;
        }

        L_indirect = brdfVal * pathtrace_nee(Ray(hitPoint, sampleDir), depth + 1, sceneParser, kdtree, false, false);
        return L_direct + L_indirect;
    }
    // 处理镜面反射
    else if (material->getSpecCoef()>0.0f) {
        Vector3f reflectDir = getReflectionDirection(ray, hit);
        Ray reflectRay(hitPoint, reflectDir);
        L_indirect = baseColor * pathtrace_nee(reflectRay, depth + 1, sceneParser, kdtree, false);
        return L_direct + L_indirect;
    }
    // 折射处理
    else {
        Vector3f refrDir;
        bool refracted = getRefractionDirection(ray, hit, 1.0f, material->getRefractiveIndex(), refrDir);
        if (refracted) {
            L_indirect = baseColor * pathtrace_nee(Ray(hitPoint, refrDir), depth + 1, sceneParser, kdtree, false, false);
        } else {
            L_indirect = Vector3f::ZERO;
        }
        return L_direct + L_indirect;
    }
}


#endif