#pragma once
#ifndef __KDTREE_HPP__
#define __KDTREE_HPP__

#include "geometry/vec.hpp"
#include <vector>
#include <memory>
#include <algorithm>
#include <functional>

namespace RayCast
{
    using namespace NRenderer;

    // KD树节点
    template<typename T>
    struct KDNode {
        T data;
        std::shared_ptr<KDNode<T>> left;
        std::shared_ptr<KDNode<T>> right;
        int axis; // 分割轴: 0=x, 1=y, 2=z
        
        KDNode(const T& data, int axis) : data(data), left(nullptr), right(nullptr), axis(axis) {}
    };

    // KD树类
    template<typename T>
    class KDTree {
    private:
        std::shared_ptr<KDNode<T>> root;
        std::function<Vec3(const T&)> getPosition;
        
        // 构建KD树
        std::shared_ptr<KDNode<T>> buildTree(std::vector<T>& points, int depth, int start, int end) {
            if (start >= end) return nullptr;
            
            int axis = depth % 3; // 循环使用x,y,z轴
            
            // 按当前轴排序指定范围内的点
            int median = (start + end) / 2;
            std::nth_element(points.begin() + start, points.begin() + median, points.begin() + end,
                [&](const T& a, const T& b) {
                    Vec3 posA = getPosition(a);
                    Vec3 posB = getPosition(b);
                    return posA[axis] < posB[axis];
                });
            
            auto node = std::make_shared<KDNode<T>>(points[median], axis);
            
            // 递归构建左右子树
            node->left = buildTree(points, depth + 1, start, median);
            node->right = buildTree(points, depth + 1, median + 1, end);
            
            return node;
        }
        
        // 范围搜索
        void rangeSearch(std::shared_ptr<KDNode<T>> node, const Vec3& position, float radius, 
                        std::vector<T>& results) const {
            if (!node) return;
            
            Vec3 nodePos = getPosition(node->data);
            float distance = glm::length(nodePos - position);
            
            if (distance <= radius) {
                results.push_back(node->data);
            }
            
            int axis = node->axis;
            float diff = position[axis] - nodePos[axis];
            
            // 优先搜索更可能包含目标点的子树
            if (diff <= 0) {
                rangeSearch(node->left, position, radius, results);
                if (diff * diff <= radius * radius) {
                    rangeSearch(node->right, position, radius, results);
                }
            } else {
                rangeSearch(node->right, position, radius, results);
                if (diff * diff <= radius * radius) {
                    rangeSearch(node->left, position, radius, results);
                }
            }
        }
        
    public:
        KDTree(const std::function<Vec3(const T&)>& getPos) : getPosition(getPos), root(nullptr) {}
        
        // 构建KD树
        void build(std::vector<T> points) {
            if (points.empty()) return;
            root = buildTree(points, 0, 0, points.size());
        }
        
        // 范围查询
        std::vector<T> queryRange(const Vec3& position, float radius) const {
            std::vector<T> results;
            if (root) {
                rangeSearch(root, position, radius, results);
            }
            return results;
        }
        
        // 检查是否为空
        bool empty() const { return root == nullptr; }
    };
}

#endif