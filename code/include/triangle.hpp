
#ifndef TRIANGLE_H
#define TRIANGLE_H

#include "object3d.hpp"
#include "utils.hpp"
#include <vecmath.h>
#include <cmath>
#include <iostream>
using namespace std;
//独立完成
// TODO: implement this class and add more fields as necessary,
class Triangle: public Object3D {

public:
    Triangle() = delete;
    
    // a b c are three vertex positions of the triangle
	Triangle( const Vector3f& a, const Vector3f& b, const Vector3f& c, Material* m) : Object3D(m) {
        vertices[0] = a;
        vertices[1] = b;
        vertices[2] = c;
		area = 0.5*Vector3f::cross(b-a, a-c).length();
        normal = Vector3f::cross(b-a, a-c).normalized();
        set_uv = false;
    }

	bool intersect(const Ray& ray, Hit& hit, float tmin, float tmax=MAX_EPS) override {
        Vector3f e1 = (vertices[0] - vertices[1]);
        Vector3f e2 = (vertices[0] - vertices[2]);
        Vector3f s = (vertices[0] - ray.getOrigin());
        
        float d = Matrix3f(ray.getDirection(), e1, e2).determinant();
        if (d == 0) return false;
        
        float t = Matrix3f(s, e1, e2).determinant() / d;
        float beta = Matrix3f(ray.getDirection(), s, e2).determinant() / d;
        float gamma = Matrix3f(ray.getDirection(), e1, s).determinant() / d;
        
        if (t > tmin && beta >= 0 && gamma >= 0 && beta <= 1 && gamma <= 1 && beta + gamma <= 1 && t <= hit.getT()) {
            // Calculate barycentric coordinates for texture mapping
            float alpha = 1.0f - beta - gamma;
            
            // Check if material has texture and apply it
            if (material && material->texture && material->texture->is_texture) {
                if (set_uv)
                {
                    Vector2f interpolated_uv = alpha * uv[0] + beta * uv[1] + gamma * uv[2];
                    Vector3f norm =  alpha * shadeNormal[0] + beta * shadeNormal[1] + gamma * shadeNormal[2];
                    
                    Vector3f textureColor = material->texture->get_color(interpolated_uv[0],interpolated_uv[1]);
                    std::cout<<textureColor[0]<<" "<<textureColor[1]<<" "<<textureColor[2]<<std::endl;
                    hit.set(t, material, -norm, textureColor);
                }
                else
                {
                    Vector3f norm =  alpha * shadeNormal[0] + beta * shadeNormal[1] + gamma * shadeNormal[2];
                    Vector2f interpolate_uv(0.0f, 0.0f);
                    interpolate_uv = Vector2f(beta, gamma);
                    Vector3f textureColor = material->texture->get_color(interpolate_uv[0],interpolate_uv[1]);
                    //std::cout<<textureColor[0]<<" "<<textureColor[1]<<" "<<textureColor[2]<<std::endl;
                    hit.set(t, material, norm, textureColor);
                }
            } else {
                // Use material's diffuse color if no texture
                if(set_norm)
                {
                    Vector3f norm =  alpha * shadeNormal[0] + beta * shadeNormal[1] + gamma * shadeNormal[2];
                    float tmp = norm[2]; 
                        norm[2]=norm[1];
                        norm[1]=tmp;
                    
                    if(Vector3f::dot(norm,normal)<0.9)
                    {   
                        float tmp = norm[2]; 
                        norm[2]=norm[1];
                        norm[1]=tmp;
                }
                    if(norm[0]<0)
                        norm= normal;
                    
                    //if (Vector3f::dot(norm,norm)<0)
                    //    norm = normal;
                    Vector3f diffuseColor = material ? material->getDiffuseColor() : Vector3f(1.0f, 1.0f, 1.0f);
                    hit.set(t, material, norm, diffuseColor);
                }
                else
                {
                Vector3f diffuseColor = material ? material->getDiffuseColor() : Vector3f(1.0f, 1.0f, 1.0f);
                hit.set(t, material, normal, diffuseColor);
                }
            }
            return true;
        }
        return false;
    }
    
	float getArea() override{
        return area;
    }
	void samplelight(Ray& r,float&pdf, Material*& m){
        float r1 = sqrtf(u(mt));
        float r2 = u(mt);
        Vector3f pos = (1-r1)*vertices[0]+r1*(1-r2)*vertices[1]+r1*r2*vertices[2];
        r = Ray(pos,normal.normalized());
		pdf = 1.0/area;
        m = material;
    }
    
    // Implementation of getBoundingBox for Triangle
    bool getBoundingBox(BoundingBox& box) override {
        // Initialize with first vertex
        Vector3f min_point = vertices[0];
        Vector3f max_point = vertices[0];
        
        // Expand to include all three vertices
        for (int i = 1; i < 3; i++) {
            for (int j = 0; j < 3; j++) {  // j = 0,1,2 for x,y,z components
                min_point[j] = std::min(min_point[j], vertices[i][j]);
                max_point[j] = std::max(max_point[j], vertices[i][j]);
            }
        }
        
        // Add small epsilon to avoid zero-thickness bounding boxes
        const float epsilon = 1e-6f;
        for (int j = 0; j < 3; j++) {
            if (max_point[j] - min_point[j] < epsilon) {
                min_point[j] -= epsilon;
                max_point[j] += epsilon;
            }
        }   
		box = BoundingBox(min_point, max_point);
        return true;
    }
    
    void Setuv(Vector2f uv0,Vector2f uv1, Vector2f uv2)
    {
        uv[0] = uv0;
        uv[1] = uv1;
        uv[2] = uv2;
        set_uv = true;

        //std::cout<<uv[0][0]<<" "<<uv[0][1]<<std::endl;
    }

    Vector3f normal;
    Vector3f vertices[3];
    Vector3f shadeNormal[3];
    
    float area;
    bool set_norm=false;

private:
    Vector2f uv[3];
    bool set_uv;
};

#endif //TRIANGLE_H