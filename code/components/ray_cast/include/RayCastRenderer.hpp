#pragma once
#ifndef __RAY_CAST_HPP__
#define __RAY_CAST_HPP__

#include "scene/Scene.hpp"
#include "Camera.hpp"
#include "intersections/intersections.hpp"
#include "shaders/ShaderCreator.hpp"
#include "KDTree.hpp"
#include <vector>
#include <memory>
#include <functional>
#include <unordered_set>

namespace RayCast
{
    using namespace NRenderer;

    // 光子结构体
    struct Photon
    {
        Vec3 position;  // 光子位置
        Vec3 direction; // 光子入射方向
        RGB power;      // 光子能量
        int bounce;     // 反弹次数
        int photonId;   // 光子唯一标识符

        Photon() = default;
        Photon(const Vec3 &pos, const Vec3 &dir, const RGB &pwr, int bnc, int id)
            : position(pos), direction(dir), power(pwr), bounce(bnc), photonId(id) {}
    };

    // 光子图类（使用KD树优化）
    class PhotonMap
    {
    private:
        std::vector<Photon> photons;
        KDTree<Photon> kdTree;

        // 位置获取函数
        static Vec3 getPhotonPosition(const Photon &photon)
        {
            return photon.position;
        }

    public:
        PhotonMap() : kdTree(getPhotonPosition) {}

        void store(const Photon &photon)
        {
            photons.push_back(photon);
        }

        // 构建KD树（在所有光子存储后调用）
        void buildKDTree()
        {
            kdTree.build(photons);
        }

        size_t size() const { return photons.size(); }
        const std::vector<Photon> &getPhotons() const { return photons; }

        // 使用KD树加速的范围查询
        std::vector<Photon> queryRange(const Vec3 &position, float radius) const
        {
            if (kdTree.empty())
            {
                // 如果KD树未构建，使用暴力搜索
                std::vector<Photon> result;
                for (const auto &photon : photons)
                {
                    if (glm::length(photon.position - position) <= radius)
                    {
                        result.push_back(photon);
                    }
                }
                return result;
            }
            return kdTree.queryRange(position, radius);
        }

        // 计算指定位置附近的光子密度 - 实验四公式实现
        RGB estimateRadiance(const Vec3 &position, const Vec3 &normal, float radius) const;

        // 自适应半径的光子密度估计
        RGB estimateRadianceAdaptive(const Vec3 &position, const Vec3 &normal, int nearestPhotons) const;

        // 统计总能量
        RGB getTotalEnergy() const;

        // 可视化光子密度
        RGB visualizePhotonDensity() const;

        // 清空光子图
        void clear() { photons.clear(); }
    };

    class RayCastRenderer
    {
    private:
        SharedScene spScene;
        Scene &scene;
        RayCast::Camera camera;
        vector<SharedShader> shaderPrograms;

        // 光子映射相关
        PhotonMap globalPhotonMap;
        int photonCount;
        int maxBounces;
        bool usePhotonMapping;
        int nextPhotonId; // 光子唯一ID计数器

    public:
        RayCastRenderer(SharedScene spScene)
            : spScene(spScene), scene(*spScene), camera(spScene->camera), photonCount(10000) // 默认光子数量
              ,
              maxBounces(5) // 最大反弹次数
              ,
              usePhotonMapping(true) // 默认使用光子映射
              ,
              nextPhotonId(0) // 光子ID从0开始
        {
        }

        ~RayCastRenderer() = default;

        using RenderResult = tuple<RGBA *, unsigned int, unsigned int>;

        RenderResult render();
        void release(const RenderResult &r);

        // 光子映射相关方法
        void buildPhotonMap();
        void tracePhoton(const Ray &ray, RGB power, int bounce, int &storedCount,
                         int photonId, std::unordered_set<int> &storedPhotonIds);
        RGB getPhotonEnergy() const { return globalPhotonMap.getTotalEnergy(); }

        // 能量守恒验证
        void verifyEnergyConservation();

        // 参数设置
        void setPhotonCount(int count) { photonCount = count; }
        void setMaxBounces(int bounces) { maxBounces = bounces; }
        void setUsePhotonMapping(bool use) { usePhotonMapping = use; }
        int getStoredPhotonCount() const { return globalPhotonMap.size(); }
        bool isLambertianMaterial(int materialIndex) const;

        bool isDielectricMaterial(int materialIndex) const;

    private:
        RGB gamma(const RGB &rgb);
        RGB trace(const Ray &r);
        HitRecord closestHit(const Ray &r);

        // 俄罗斯轮盘赌

        bool russianRoulette(int bounce);
        // 随机方向生成
        Vec3 randomHemisphereDirection(const Vec3 &normal);
        Vec3 randomCosineWeightedDirection(const Vec3 &normal);

        // 光子可视化
        RGB renderPhotonVisualization();

        // 场景分析
        void analyzeScene();

        // 获取材质反射率
        RGB getMaterialReflectance(int materialIndex) const;
    };
}

#endif