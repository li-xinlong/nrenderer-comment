#include "KDTree.hpp"
#include <algorithm>
#include <numeric>
#include <iostream>

namespace RayCast
{
    KDTree::KDTree(SharedScene scene, int maxDepth, int leafThreshold)
        : spScene(scene), maxDepth(maxDepth), leafThreshold(leafThreshold), 
          root(nullptr)
    {
        stats.reset();
    }
    
    void KDTree::build() {
        std::vector<int> triangles, spheres, planes;
        
        for (int i = 0; i < spScene->triangleBuffer.size(); i++)
            triangles.push_back(i);
        for (int i = 0; i < spScene->sphereBuffer.size(); i++)
            spheres.push_back(i);
        for (int i = 0; i < spScene->planeBuffer.size(); i++)
            planes.push_back(i);
        
        AABB sceneBounds = computeBounds(triangles, spheres, planes);
        
        std::cout << "Building KD-Tree with " 
                  << triangles.size() << " triangles, "
                  << spheres.size() << " spheres, "
                  << planes.size() << " planes" << std::endl;
        
        root = buildRecursive(sceneBounds, triangles, spheres, planes, 0);
        
        std::cout << "KD-Tree built: " << getNodeCount() << " nodes, " 
                  << getLeafCount() << " leaves" << std::endl;
    }
    
    std::shared_ptr<KDNode> KDTree::buildRecursive(const AABB& bounds,
                                                  std::vector<int> triangles,
                                                  std::vector<int> spheres, 
                                                  std::vector<int> planes,
                                                  int depth) {
        auto node = std::make_shared<KDNode>();
        node->bounds = bounds;
        node->isLeaf = true;
        
        int totalObjects = triangles.size() + spheres.size() + planes.size();
        
        if (depth >= maxDepth || totalObjects <= leafThreshold) {
            node->triangles = std::move(triangles);
            node->spheres = std::move(spheres);
            node->planes = std::move(planes);
            return node;
        }
        
        node->axis = chooseSplitAxis(bounds);
        node->splitPos = findMedianSplit(node->axis, triangles, spheres, planes);
        
        std::vector<int> leftTriangles, rightTriangles;
        std::vector<int> leftSpheres, rightSpheres;
        std::vector<int> leftPlanes, rightPlanes;
        
        classifyObjects(node->axis, node->splitPos,
                       triangles, spheres, planes,
                       leftTriangles, rightTriangles,
                       leftSpheres, rightSpheres,
                       leftPlanes, rightPlanes);
        
        if (leftTriangles.size() + leftSpheres.size() + leftPlanes.size() == totalObjects ||
            rightTriangles.size() + rightSpheres.size() + rightPlanes.size() == totalObjects) {
            node->triangles = std::move(triangles);
            node->spheres = std::move(spheres);
            node->planes = std::move(planes);
            return node;
        }
        
        AABB leftBounds = bounds;
        AABB rightBounds = bounds;
        
        switch (node->axis) {
            case 0:
                leftBounds.max.x = node->splitPos;
                rightBounds.min.x = node->splitPos;
                break;
            case 1:
                leftBounds.max.y = node->splitPos;
                rightBounds.min.y = node->splitPos;
                break;
            case 2:
                leftBounds.max.z = node->splitPos;
                rightBounds.min.z = node->splitPos;
                break;
        }
        
        node->isLeaf = false;
        
        if (!leftTriangles.empty() || !leftSpheres.empty() || !leftPlanes.empty()) {
            node->left = buildRecursive(leftBounds, 
                                      std::move(leftTriangles), 
                                      std::move(leftSpheres), 
                                      std::move(leftPlanes), 
                                      depth + 1);
        }
        
        if (!rightTriangles.empty() || !rightSpheres.empty() || !rightPlanes.empty()) {
            node->right = buildRecursive(rightBounds, 
                                       std::move(rightTriangles), 
                                       std::move(rightSpheres), 
                                       std::move(rightPlanes), 
                                       depth + 1);
        }
        
        return node;
    }
    
    int KDTree::chooseSplitAxis(const AABB& bounds) {
        Vec3 extent = bounds.max - bounds.min;
        if (extent.x >= extent.y && extent.x >= extent.z) return 0;
        if (extent.y >= extent.z) return 1;
        return 2;
    }
    
    float KDTree::findMedianSplit(int axis, 
                                 const std::vector<int>& triangles,
                                 const std::vector<int>& spheres,
                                 const std::vector<int>& planes) {
        std::vector<float> positions;
        
        for (int idx : triangles) {
            const auto& tri = spScene->triangleBuffer[idx];
            positions.push_back(tri.v1[axis]);
            positions.push_back(tri.v2[axis]);
            positions.push_back(tri.v3[axis]);
        }
        
        for (int idx : spheres) {
            const auto& sphere = spScene->sphereBuffer[idx];
            positions.push_back(sphere.position[axis]);
        }
        
        for (int idx : planes) {
            const auto& plane = spScene->planeBuffer[idx];
            positions.push_back(plane.position[axis]);
        }
        
        if (positions.empty()) {
            return 0.0f;
        }
        
        std::sort(positions.begin(), positions.end());
        return positions[positions.size() / 2];
    }
    
    void KDTree::classifyObjects(int axis, float splitPos,
                                const std::vector<int>& triangles,
                                const std::vector<int>& spheres,
                                const std::vector<int>& planes,
                                std::vector<int>& leftTriangles, std::vector<int>& rightTriangles,
                                std::vector<int>& leftSpheres, std::vector<int>& rightSpheres,
                                std::vector<int>& leftPlanes, std::vector<int>& rightPlanes) {
        for (int idx : triangles) {
            const auto& tri = spScene->triangleBuffer[idx];
            AABB triBounds = computeTriangleBounds(tri);
            
            if (triBounds.max[axis] <= splitPos) {
                leftTriangles.push_back(idx);
            } else if (triBounds.min[axis] >= splitPos) {
                rightTriangles.push_back(idx);
            } else {
                leftTriangles.push_back(idx);
                rightTriangles.push_back(idx);
            }
        }
        
        for (int idx : spheres) {
            const auto& sphere = spScene->sphereBuffer[idx];
            AABB sphereBounds = computeSphereBounds(sphere);
            
            if (sphereBounds.max[axis] <= splitPos) {
                leftSpheres.push_back(idx);
            } else if (sphereBounds.min[axis] >= splitPos) {
                rightSpheres.push_back(idx);
            } else {
                leftSpheres.push_back(idx);
                rightSpheres.push_back(idx);
            }
        }
        
        for (int idx : planes) {
            leftPlanes.push_back(idx);
            rightPlanes.push_back(idx);
        }
    }
    
    AABB KDTree::computeBounds(const std::vector<int>& triangles,
                              const std::vector<int>& spheres,
                              const std::vector<int>& planes) {
        AABB bounds;
        
        for (int idx : triangles) {
            bounds.expand(computeTriangleBounds(spScene->triangleBuffer[idx]));
        }
        
        for (int idx : spheres) {
            bounds.expand(computeSphereBounds(spScene->sphereBuffer[idx]));
        }
        
        for (int idx : planes) {
            bounds.expand(computePlaneBounds(spScene->planeBuffer[idx]));
        }
        
        return bounds;
    }
    
    HitRecord KDTree::closestHit(const Ray& ray) const {
        // 统计：处理的光线数量
        stats.raysProcessed++;
        
        HitRecord closestHit = getMissRecord();
        float closestDist = FLOAT_INF;
        
        if (root && Intersection::xAABB(ray, root->bounds)) {
            queryRecursive(root, ray, closestHit, closestDist);
        }
        
        return closestHit;
    }
    
    void KDTree::queryRecursive(const std::shared_ptr<KDNode>& node, 
                               const Ray& ray, HitRecord& closest, float& closestDist) const {
        // 统计：节点访问次数
        stats.totalNodesVisited++;
        
        // 统计：光线-AABB检测次数
        stats.totalRayAABBTests++;
        
        // 检查光线是否与节点包围盒相交
        if (!Intersection::xAABB(ray, node->bounds, 0.0f, closestDist)) {
            // 统计：提前退出（AABB检测失败）
            stats.totalEarlyOuts++;
            return;
        }
        
        if (node->isLeaf) {
            // 统计：叶节点访问次数
            stats.totalLeafNodesVisited++;
            
            // 检查叶节点中的三角形
            for (int idx : node->triangles) {
                // 统计：光线-三角形检测次数
                stats.totalRayTriangleTests++;
                
                auto hitRecord = Intersection::xTriangle(ray, spScene->triangleBuffer[idx], 0.01f, closestDist);
                if (hitRecord && hitRecord->t < closestDist) {
                    closestDist = hitRecord->t;
                    closest = hitRecord;
                }
            }
            
            // 检查叶节点中的球体
            for (int idx : node->spheres) {
                // 统计：光线-球体检测次数
                stats.totalRaySphereTests++;
                
                auto hitRecord = Intersection::xSphere(ray, spScene->sphereBuffer[idx], 0.01f, closestDist);
                if (hitRecord && hitRecord->t < closestDist) {
                    closestDist = hitRecord->t;
                    closest = hitRecord;
                }
            }
            
            // 检查叶节点中的平面
            for (int idx : node->planes) {
                // 统计：光线-平面检测次数
                stats.totalRayPlaneTests++;
                
                auto hitRecord = Intersection::xPlane(ray, spScene->planeBuffer[idx], 0.01f, closestDist);
                if (hitRecord && hitRecord->t < closestDist) {
                    closestDist = hitRecord->t;
                    closest = hitRecord;
                }
            }
        } else {
            // 决定遍历顺序：先近后远
            bool nearFirst = (ray.direction[node->axis] >= 0);
            auto firstChild = nearFirst ? node->left : node->right;
            auto secondChild = nearFirst ? node->right : node->left;
            
            if (firstChild) {
                queryRecursive(firstChild, ray, closest, closestDist);
            }
            if (secondChild) {
                queryRecursive(secondChild, ray, closest, closestDist);
            }
        }
    }
    
    int KDTree::getNodeCount() const {
        int totalNodes = 0, leafNodes = 0;
        countNodes(root, totalNodes, leafNodes);
        return totalNodes;
    }
    
    int KDTree::getLeafCount() const {
        int totalNodes = 0, leafNodes = 0;
        countNodes(root, totalNodes, leafNodes);
        return leafNodes;
    }
    
    void KDTree::countNodes(const std::shared_ptr<KDNode>& node, int& totalNodes, int& leafNodes) const {
        if (!node) return;
        
        totalNodes++;
        if (node->isLeaf) {
            leafNodes++;
        } else {
            countNodes(node->left, totalNodes, leafNodes);
            countNodes(node->right, totalNodes, leafNodes);
        }
    }
}