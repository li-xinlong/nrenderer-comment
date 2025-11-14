#pragma once
#ifndef __AABB_HPP__
#define __AABB_HPP__

#include "geometry/vec.hpp"
#include "Ray.hpp"
#include <limits>

// 前向声明几何体类型
namespace NRenderer {
    struct Sphere;
    struct Triangle;
    struct Plane;
}

namespace RayCast
{
    using namespace NRenderer;

    // 轴对齐包围盒 (AABB)
    struct AABB {
        Vec3 min;
        Vec3 max;

        AABB() : min(std::numeric_limits<float>::max()), 
                 max(std::numeric_limits<float>::lowest()) {}

        AABB(const Vec3& min, const Vec3& max) : min(min), max(max) {}

        void expand(const Vec3& point) {
            min = glm::min(min, point);
            max = glm::max(max, point);
        }

        void expand(const AABB& other) {
            min = glm::min(min, other.min);
            max = glm::max(max, other.max);
        }

        Vec3 center() const {
            return (min + max) * 0.5f;
        }

        float diagonal() const {
            return glm::length(max - min);
        }

        float surfaceArea() const {
            Vec3 extent = max - min;
            return 2.0f * (extent.x * extent.y + extent.x * extent.z + extent.y * extent.z);
        }

        bool contains(const Vec3& point) const {
            return point.x >= min.x && point.x <= max.x &&
                   point.y >= min.y && point.y <= max.y &&
                   point.z >= min.z && point.z <= max.z;
        }
    };

    // 前向声明函数
    AABB computeSphereBounds(const Sphere& sphere);
    AABB computeTriangleBounds(const Triangle& triangle);
    AABB computePlaneBounds(const Plane& plane);
}

#endif