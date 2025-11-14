#pragma once
#ifndef __KDTREE_HPP__
#define __KDTREE_HPP__

#include "AABB.hpp"
#include "scene/Scene.hpp"
#include "Ray.hpp"
#include "intersections/HitRecord.hpp"
#include "intersections/intersections.hpp"
#include <vector>
#include <memory>
#include <algorithm>

namespace RayCast
{
    using namespace NRenderer;

    struct KDNode {
        AABB bounds;
        int axis;
        float splitPos;
        std::shared_ptr<KDNode> left;
        std::shared_ptr<KDNode> right;
        
        std::vector<int> triangles;
        std::vector<int> spheres;
        std::vector<int> planes;
        
        bool isLeaf;
        
        KDNode() : axis(0), splitPos(0.0f), isLeaf(true) {}
    };

    class KDTree {
    private:
        std::shared_ptr<KDNode> root;
        SharedScene spScene;
        
        int maxDepth;
        int leafThreshold;
        
        // 性能统计结构
        mutable struct Statistics {
            int totalNodesVisited = 0;           // 节点访问总次数
            int totalLeafNodesVisited = 0;       // 叶节点访问次数
            int totalRayAABBTests = 0;           // 光线-AABB检测次数
            int totalRayTriangleTests = 0;       // 光线-三角形检测次数
            int totalRaySphereTests = 0;         // 光线-球体检测次数
            int totalRayPlaneTests = 0;          // 光线-平面检测次数
            int totalEarlyOuts = 0;              // 提前退出次数（AABB检测失败）
            int raysProcessed = 0;               // 处理的光线数量
            
            // 计算总物体检测次数
            int totalObjectTests() const {
                return totalRayTriangleTests + totalRaySphereTests + totalRayPlaneTests;
            }
            
            // 重置所有统计
            void reset() {
                totalNodesVisited = 0;
                totalLeafNodesVisited = 0;
                totalRayAABBTests = 0;
                totalRayTriangleTests = 0;
                totalRaySphereTests = 0;
                totalRayPlaneTests = 0;
                totalEarlyOuts = 0;
                raysProcessed = 0;
            }
        } stats;

        std::shared_ptr<KDNode> buildRecursive(const AABB& bounds,
                                              std::vector<int> triangles,
                                              std::vector<int> spheres, 
                                              std::vector<int> planes,
                                              int depth);
        
        int chooseSplitAxis(const AABB& bounds);
        float findMedianSplit(int axis, 
                             const std::vector<int>& triangles,
                             const std::vector<int>& spheres,
                             const std::vector<int>& planes);
        
        void classifyObjects(int axis, float splitPos,
                            const std::vector<int>& triangles,
                            const std::vector<int>& spheres,
                            const std::vector<int>& planes,
                            std::vector<int>& leftTriangles, std::vector<int>& rightTriangles,
                            std::vector<int>& leftSpheres, std::vector<int>& rightSpheres,
                            std::vector<int>& leftPlanes, std::vector<int>& rightPlanes);
        
        AABB computeBounds(const std::vector<int>& triangles,
                          const std::vector<int>& spheres,
                          const std::vector<int>& planes);

        void queryRecursive(const std::shared_ptr<KDNode>& node, 
                           const Ray& ray, HitRecord& closest, float& closestDist) const;

    public:
        KDTree(SharedScene scene, int maxDepth = 20, int leafThreshold = 5);
        
        void build();
        
        HitRecord closestHit(const Ray& ray) const;
        
        int getNodeCount() const;
        int getLeafCount() const;
        
        // 统计相关方法
        void resetStats() const { stats.reset(); }
        const Statistics& getStats() const { return stats; }
        void incrementRaysProcessed() const { stats.raysProcessed++; }
        
    private:
        void countNodes(const std::shared_ptr<KDNode>& node, int& totalNodes, int& leafNodes) const;
    };
}

#endif