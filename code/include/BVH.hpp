#ifndef KDTREE_H
#define KDTREE_H

#include "utils.hpp"
#include "object3d.hpp"
#include "group.hpp"
#include "ray.hpp"
#include "hit.hpp"
#include "bounding_box.hpp"
#include <vector>
#include <algorithm>
#include <memory>
#include <stack>
//独立完成
// KD-Tree Node structure
struct KDNode {
    BoundingBox bbox;
    std::shared_ptr<KDNode> left;
    std::shared_ptr<KDNode> right;
    std::vector<Object3D*> objects;  // Objects stored in this node
    int split_axis;                  // 0=x, 1=y, 2=z
    float split_pos;                 // Split position along axis
    
    KDNode() : split_axis(-1), split_pos(0.0f) {}
    
    bool isLeaf() const {
        return left == nullptr && right == nullptr;
    }
};

class KDTree : public Object3D {
private:
    std::shared_ptr<KDNode> root;
    std::vector<Object3D*> all_objects;
    int max_depth;
    int max_objects_per_leaf;
    
    // Helper function to compute bounding box for objects
    BoundingBox computeBoundingBox(const std::vector<Object3D*>& objs) {
        if (objs.empty()) return BoundingBox();
        
        BoundingBox bbox;
        objs[0]->getBoundingBox(bbox);
        for (size_t i = 1; i < objs.size(); i++) {
            BoundingBox new_box;
            objs[i]->getBoundingBox(new_box);
            bbox = BoundingBox::Union(bbox, new_box);
        }
        return bbox;
    }
    
    // Check if object intersects with bounding box
    bool objectIntersectsBox(Object3D* obj, const BoundingBox& box) {
        BoundingBox objBox;
        obj->getBoundingBox(objBox);
        return BoundingBox::Intersects(objBox, box);
    }
    
    // Recursive KD-Tree construction
    std::shared_ptr<KDNode> buildKDTree(std::vector<Object3D*>& objs, 
                                       const BoundingBox& bbox, 
                                       int depth = 0) {
        auto node = std::make_shared<KDNode>();
        node->bbox = bbox;
        
        // Base cases for creating leaf nodes
        if (depth >= max_depth || (int)objs.size() <= max_objects_per_leaf) {
            node->objects = objs;
            return node;
        }
        
        // Choose split axis (cycle through x, y, z)
        int axis = depth % 3;
        node->split_axis = axis;
        
        // Find the best split position using median of object centers
        std::vector<float> centers;
        centers.reserve(objs.size());
        
        for (Object3D* obj : objs) {
            BoundingBox box;
            obj->getBoundingBox(box);
            Vector3f center = box.getCenter();
            centers.push_back(center[axis]);
        }
        
        std::sort(centers.begin(), centers.end());
        
        // Use median as split position
        float split_pos;
        if (centers.size() % 2 == 0) {
            split_pos = (centers[centers.size()/2 - 1] + centers[centers.size()/2]) / 2.0f;
        } else {
            split_pos = centers[centers.size()/2];
        }
        
        node->split_pos = split_pos;
        
        // Create left and right bounding boxes
        BoundingBox leftBox = bbox;
        BoundingBox rightBox = bbox;
        
        Vector3f leftMax = leftBox.getMax();
        Vector3f rightMin = rightBox.getMin();
        
        leftMax[axis] = split_pos;
        rightMin[axis] = split_pos;
        
        leftBox = BoundingBox(leftBox.getMin(), leftMax);
        rightBox = BoundingBox(rightMin, rightBox.getMax());
        
        // Distribute objects to left and right
        std::vector<Object3D*> leftObjects, rightObjects;
        
        for (Object3D* obj : objs) {
            BoundingBox box;
            obj->getBoundingBox(box);
            Vector3f center = box.getCenter();
            
            // Objects can be in both sides if they straddle the split plane
            if (objectIntersectsBox(obj, leftBox)) {
                leftObjects.push_back(obj);
            }
            if (objectIntersectsBox(obj, rightBox)) {
                rightObjects.push_back(obj);
            }
        }
        
        // If we couldn't split effectively, make this a leaf
        if (leftObjects.empty() || rightObjects.empty() || 
            (leftObjects.size() == objs.size() && rightObjects.size() == objs.size())) {
            node->objects = objs;
            return node;
        }
        
        // Recursively build left and right subtrees
        if (!leftObjects.empty()) {
            node->left = buildKDTree(leftObjects, leftBox, depth + 1);
        }
        if (!rightObjects.empty()) {
            node->right = buildKDTree(rightObjects, rightBox, depth + 1);
        }
        
        return node;
    }
    
    // Recursive intersection test with early termination
    bool intersectKDTree(const std::shared_ptr<KDNode>& node, const Ray& r, Hit& h, float tmin) {
        if (!node) return false;
        
        // Test intersection with bounding box first
        float t_near, t_far;
        if (!node->bbox.intersect(r, tmin, h.getT(), t_near, t_far)) {
            return false;
        }
        
        // If this is beyond our current best hit, skip
        if (t_near > h.getT()) {
            return false;
        }
        
        // If leaf node, test intersection with all objects
        if (node->isLeaf()) {
            bool hit_any = false;
            for (Object3D* obj : node->objects) {
                if (obj->intersect(r, h, tmin)) {
                    hit_any = true;
                }
            }
            return hit_any;
        }
        
        // Internal node: determine traversal order
        Vector3f origin = r.getOrigin();
        Vector3f direction = r.getDirection();
        
        // Determine which child to visit first based on ray direction
        bool ray_going_left = origin[node->split_axis] < node->split_pos || 
                             (origin[node->split_axis] == node->split_pos && direction[node->split_axis] <= 0);
        
        std::shared_ptr<KDNode> first_child = ray_going_left ? node->left : node->right;
        std::shared_ptr<KDNode> second_child = ray_going_left ? node->right : node->left;
        
        bool hit_first = false;
        if (first_child) {
            hit_first = intersectKDTree(first_child, r, h, tmin);
        }
        
        // Early termination: if we hit something in the first child and the hit is
        // closer than the split plane, we don't need to check the second child
        if (hit_first) {
            float t_split = (node->split_pos - origin[node->split_axis]) / direction[node->split_axis];
            if (h.getT() < t_split) {
                return true;
            }
        }
        
        bool hit_second = false;
        if (second_child) {
            hit_second = intersectKDTree(second_child, r, h, tmin);
        }
        
        return hit_first || hit_second;
    }

public:
    KDTree(int max_d = 20, int max_objs = 10) 
        : root(nullptr), max_depth(max_d), max_objects_per_leaf(max_objs) {}
    
    // Constructor from Group
    explicit KDTree(const Group& group, int max_d = 20, int max_objs = 10) 
        : max_depth(max_d), max_objects_per_leaf(max_objs) {
        all_objects = group.getObjects();
        if (!all_objects.empty()) {
            build();
        }
    }
    
    // Constructor from vector of objects
    explicit KDTree(const std::vector<Object3D*>& objs, int max_d = 20, int max_objs = 10) 
        : all_objects(objs), max_depth(max_d), max_objects_per_leaf(max_objs) {
        if (!all_objects.empty()) {
            build();
        }
    }
    
    void build() {
        if (all_objects.empty()) return;
        
        BoundingBox sceneBBox = computeBoundingBox(all_objects);
        std::vector<Object3D*> buildObjects = all_objects;
        root = buildKDTree(buildObjects, sceneBBox);
    }
    
    bool intersect(const Ray& r, Hit& h, float tmin,float tmax=MAX_EPS) override {
        if (!root) return false;
        return intersectKDTree(root, r, h, tmin);
    }
    
    void samplelight(Ray& ray, float& pdf, Material*& m) override {
        // For sampling, we'll use the same approach as Group
        float total_area = 0;
        for (auto obj : all_objects) {
            if (obj->getMaterial()->getEmission().length() > EPSILON) {
                total_area += obj->getArea();
            }
        }
        
        float p = u(mt) * total_area;
        float area = 0;
        for (auto obj : all_objects) {
            if (obj->getMaterial()->getEmission().length() > EPSILON) {
                area += obj->getArea();
                if (p < area) {
                    obj->samplelight(ray, pdf, m);
                    return;
                }
            }
        }
    }
    
    bool getBoundingBox(BoundingBox& box) override {
    if (root) {
        box = root->bbox;
        return true;
    }
    return false; // No valid bounding box
    }

    
    float getArea() override {
        float totalArea = 0;
        for (auto obj : all_objects) {
            totalArea += obj->getArea();
        }
        return totalArea;
    }
    
    // Utility functions
    void printStats() const {
        if (!root) {
            std::cout << "KD-Tree is empty" << std::endl;
            return;
        }
        
        std::cout << "KD-Tree Stats:" << std::endl;
        std::cout << "  Total objects: " << all_objects.size() << std::endl;
        std::cout << "  Max depth: " << max_depth << std::endl;
        std::cout << "  Max objects per leaf: " << max_objects_per_leaf << std::endl;
        std::cout << "  Root bounding box: " << root->bbox.toString() << std::endl;
        
        // Count actual depth and leaf nodes
        int actualDepth = 0;
        int leafCount = 0;
        int totalNodes = 0;
        countStats(root, 0, actualDepth, leafCount, totalNodes);
        
        std::cout << "  Actual depth: " << actualDepth << std::endl;
        std::cout << "  Leaf nodes: " << leafCount << std::endl;
        std::cout << "  Total nodes: " << totalNodes << std::endl;
    }
    
    // Add objects to KD-Tree
    void addObject(Object3D* obj) {
        all_objects.push_back(obj);
    }
    
    void clear() {
        all_objects.clear();
        root = nullptr;
    }
    
    void rebuild() {
        build();
    }
    
    // Set construction parameters
    void setMaxDepth(int depth) { max_depth = depth; }
    void setMaxObjectsPerLeaf(int objs) { max_objects_per_leaf = objs; }

private:
    void countStats(const std::shared_ptr<KDNode>& node, int depth, 
                   int& maxDepth, int& leafCount, int& totalNodes) const {
        if (!node) return;
        
        totalNodes++;
        maxDepth = std::max(maxDepth, depth);
        
        if (node->isLeaf()) {
            leafCount++;
        } else {
            countStats(node->left, depth + 1, maxDepth, leafCount, totalNodes);
            countStats(node->right, depth + 1, maxDepth, leafCount, totalNodes);
        }
    }
};

#endif // KDTREE_H