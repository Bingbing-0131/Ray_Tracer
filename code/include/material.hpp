#ifndef MATERIAL_H
#define MATERIAL_H
#define MAX(a,b) (((a) > (b)) ? (a) : (b) )
#include <cassert>
#include <vecmath.h>

#include "ray.hpp"
#include "hit.hpp"
#include <iostream>
#include "texture.hpp"
#include "utils.hpp"
// TODO: Implement Shade function that computes Phong introduced in class.
// 独立完成
class Material {
public:

    explicit Material(const Vector3f &d_color, const Vector3f &s_color = Vector3f::ZERO, float s = 0) :
            diffuseColor(d_color), specularColor(s_color), shininess(s), 
            diffCoef(1.0f), specCoef(0.0f), refractivity(0.0f), refractiveIndex(1.0f), emission(Vector3f::ZERO),texture(new Texture()), useck(false),
            roughness(0.0f), bssrdf(false) {  // 修正：添加初始化
    }
                

    virtual ~Material() = default;

    Vector3f getDiffuseColor() const {
        return diffuseColor;
    }


    float getDiffCoef() const {
        return diffCoef;
    }

    float getSpecCoef() const {
        return specCoef;
    }

    float getRefractivity() const {
        return refractivity;
    }

    Vector3f getReflectiveColor() const {
        return diffuseColor; 
    }

    float getReflectionCoeff() const {
        return specCoef; 
    }

    Vector3f getRefractiveColor() const {
        return diffuseColor; 
    }

    float getRefractionCoeff() const {
        return refractivity; 
    }

    float getRefractiveIndex() const {
        return refractiveIndex;
    }
    Vector3f getEmission() const {
        return emission;
    }


    Vector3f Shade(const Ray &ray, const Hit &hit,
        const Vector3f &dirToLight, const Vector3f &lightColor) {

        Vector3f shaded = Vector3f::ZERO;
        Vector3f n = hit.getNormal().normalized();         
        Vector3f l = dirToLight.normalized();                
        Vector3f v = -ray.getDirection().normalized();        
        Vector3f r = (2 * Vector3f::dot(n, l) * n - l).normalized();  

        float NdotL = std::max(Vector3f::dot(n, l), 0.0f);
        Vector3f diffuse = diffuseColor * lightColor * NdotL;

        float RdotV = std::max(Vector3f::dot(r, v), 0.0f);
        float specularIntensity = pow(RdotV, shininess);
        Vector3f specular = specularColor * lightColor * specularIntensity;
        shaded = diffuse + specular;

        return shaded;
    }

    Texture* texture;
    float roughness;
    bool useck;
    bool bssrdf;
    Vector3f sphere_center = Vector3f(73,46.5,78);
    float sphere_radius = 16.5;
    
protected:
    Vector3f diffuseColor;
    Vector3f specularColor;
    Vector3f emission;
    float shininess;
    float diffCoef;
    float specCoef;
    float refractivity;
    float refractiveIndex;

};


#endif // MATERIAL_H
