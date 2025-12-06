#include "mesh.hpp"
#include "tiny_obj_loader.h"
#include "BVH.hpp"  // Include your existing BVH/KDTree header
#include <fstream>
#include <iostream>
#include <algorithm>
#include <cstdlib>
#include <utility>
#include <sstream>
//tiny obj loader借鉴他人教程
Mesh::Mesh(const char *filename, Material *material) : Object3D(material) {
    // Use existing tiny_obj_loader code
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string warn, err;
    std::string basepath = std::string(filename).substr(0, std::string(filename).find_last_of("/\\") + 1);
    bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, filename, basepath.c_str());
    
    if (!err.empty()) {
        std::cerr << err << '\n';
    }
    if (!ret) {
        std::cerr << "Failed to load/parse " << filename << ".\n";
        return;
    }
    
    // Load vertices
    for (size_t vi = 0; vi < attrib.vertices.size(); vi += 3) {
        v.push_back(Vector3f(attrib.vertices[vi], attrib.vertices[vi + 1], attrib.vertices[vi + 2]));
    }
    
    // Load normals
    for (size_t ni = 0; ni < attrib.normals.size(); ni += 3) {
        n.push_back(Vector3f(attrib.normals[ni], attrib.normals[ni + 1], attrib.normals[ni + 2]).normalized());
    }
    
    // Load texture coordinates
    for (size_t ti = 0; ti < attrib.texcoords.size(); ti += 2) {
        uv.push_back(Vector2f(attrib.texcoords[ti], attrib.texcoords[ti + 1]));
    }
    
    // Load faces and create triangles
    for (size_t s = 0; s < shapes.size(); s++) {
        size_t index_offset = 0;
        for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
            int fv = shapes[s].mesh.num_face_vertices[f];
            TriangleIndex triIndex,normIndex,texIndex;
            for (size_t v = 0; v < fv; v++) {
                tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
                triIndex[v] = idx.vertex_index;
                normIndex[v] = idx.normal_index;
                texIndex[v] = idx.texcoord_index;
            }
            t.push_back(triIndex);
            
            Triangle* triangle = new Triangle(v[triIndex[0]], v[triIndex[1]], v[triIndex[2]], material);
            
            // Set shade normals if available
            if (n.size() > 0) {
                triangle->shadeNormal[0] = n[triIndex[0]];
                triangle->shadeNormal[1] = n[triIndex[1]];
                triangle->shadeNormal[2] = n[triIndex[2]];
                triangle->set_norm= true;
            }

            if (uv.size()>0)
            {
                Vector2f uv0 = uv[texIndex[0]];
                Vector2f uv1 = uv[texIndex[1]];
                Vector2f uv2 = uv[texIndex[2]];
                triangle->Setuv(uv0,uv1,uv2);
                //std::cout<<"set"<<std::endl;
            }

            
            triangles.push_back(triangle);
            index_offset += fv;
        }
    }
    
    std::cout << "Loaded mesh with " << v.size() << " vertices and " 
              << triangles.size() << " triangles" << std::endl;
    
    // Create BVH/KDTree from the triangles for fast intersection
    std::vector<Object3D*> triangle_objects;
    for (Triangle* tri : triangles) {
        triangle_objects.push_back(tri);
    }
    
    // Create the acceleration structure
    bvh_tree = new KDTree(triangle_objects);  // or use BVH if you have that class
    std::cout << "Built acceleration structure for mesh" << std::endl;
}

Mesh::~Mesh() {
    // Clean up triangles
    for (auto triangle : triangles) {
        delete triangle;
    }
    triangles.clear();
    
    // Clean up BVH
    if (bvh_tree) {
        delete bvh_tree;
        bvh_tree = nullptr;
    }
}

bool Mesh::intersect(const Ray &r, Hit &h, float tmin, float tmax) {
    // Use BVH for fast intersection
    if (bvh_tree) {
        return bvh_tree->intersect(r, h, tmin);
    }
    
    // Fallback to linear search if BVH not available
    bool result = false;
    for (auto triangle : triangles) {
        if (triangle) {
            if (triangle->intersect(r, h, tmin, tmax)) {
                result = true;
                tmax = h.getT(); // Update for closest hit
            }
        }
    }
    return result;
}

bool Mesh::getBoundingBox(BoundingBox& box) {
    // Use BVH's bounding box if available
    if (bvh_tree) {
        return bvh_tree->getBoundingBox(box);  // Pass the box parameter and return bool
    }
    
    // Fallback: compute from vertices
    if (v.empty()) {
        return false;  // Return false for empty mesh, don't assign to box
    }
    
    Vector3f min_point = v[0];
    Vector3f max_point = v[0];
    
    for (size_t i = 1; i < v.size(); i++) {
        for (int j = 0; j < 3; j++) {
            min_point[j] = std::min(min_point[j], v[i][j]);
            max_point[j] = std::max(max_point[j], v[i][j]);
        }
    }
    
    box = BoundingBox(min_point, max_point);
    return true;  // Return true to indicate success
}

float Mesh::getArea() {
    float total_area = 0;
    for (auto triangle : triangles) {
        total_area += triangle->getArea();
    }
    return total_area;
}

void Mesh::samplelight(Ray& r, float& pdf, Material*& m) {
    // Use BVH's sampling if available
    if (bvh_tree) {
        bvh_tree->samplelight(r, pdf, m);
        return;
    }
    
    // Fallback sampling
    float total_area = 0;
    for (auto triangle : triangles) {
        total_area += triangle->getArea();
    }
    float p = u(mt) * total_area;
    float area = 0;
    for (auto triangle : triangles) {
        area += triangle->getArea();
        if (p < area) {
            triangle->samplelight(r, pdf, m);
            break;
        }
    }
}

void Mesh::computeNormal() {
    n.resize(t.size());
    for (int triId = 0; triId < (int) t.size(); ++triId) {
        TriangleIndex& triIndex = t[triId];
        Vector3f a = v[triIndex[1]] - v[triIndex[0]];
        Vector3f b = v[triIndex[2]] - v[triIndex[0]];
        b = Vector3f::cross(a, b);
        n[triId] = b / b.length();
    }
}