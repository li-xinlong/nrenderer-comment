#include "AABB.hpp"
#include "scene/Scene.hpp"  // 包含Scene.hpp以获取Sphere、Triangle、Plane的完整定义

namespace RayCast
{
    AABB computeSphereBounds(const Sphere& sphere) {
        Vec3 radiusVec(sphere.radius, sphere.radius, sphere.radius);
        return AABB(sphere.position - radiusVec, sphere.position + radiusVec);
    }

    AABB computeTriangleBounds(const Triangle& triangle) {
        AABB bounds;
        bounds.expand(triangle.v1);
        bounds.expand(triangle.v2);
        bounds.expand(triangle.v3);
        return bounds;
    }

    AABB computePlaneBounds(const Plane& plane) {
        // 对于无限平面，返回一个很大的包围盒
        float large = 10000.0f;
        return AABB(Vec3(-large, -large, -large), Vec3(large, large, large));
    }

    namespace Intersection {
        bool xAABB(const Ray& ray, const AABB& aabb, float tMin, float tMax) {
            Vec3 invDir = 1.0f / ray.direction;
            Vec3 t0 = (aabb.min - ray.origin) * invDir;
            Vec3 t1 = (aabb.max - ray.origin) * invDir;
            
            Vec3 tmin = glm::min(t0, t1);
            Vec3 tmax = glm::max(t0, t1);
            
            float t_min = glm::max(glm::max(tmin.x, tmin.y), tmin.z);
            float t_max = glm::min(glm::min(tmax.x, tmax.y), tmax.z);
            
            return t_max >= t_min && t_max > tMin && t_min < tMax;
        }
    }
}