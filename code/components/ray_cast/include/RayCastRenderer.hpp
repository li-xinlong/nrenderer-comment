#pragma once
#ifndef __RAY_CAST_HPP__
#define __RAY_CAST_HPP__

#include "scene/Scene.hpp"
#include "Camera.hpp"
#include "intersections/intersections.hpp"
#include "shaders/ShaderCreator.hpp"
#include "KDTree.hpp"  // 添加KDTree包含
#include <chrono>      // 添加时间库
#include <iostream>    // 添加iostream用于输出

namespace RayCast
{
    using namespace NRenderer;

    class RayCastRenderer
    {
    private:
        SharedScene spScene;
        Scene& scene;
        RayCast::Camera camera;
        vector<SharedShader> shaderPrograms;
        std::unique_ptr<KDTree> kdTree;      // 添加KDTree成员
        bool useAcceleration;                // 是否使用加速结构

    public:
        // 构造函数
        // spScene: 场景指针
        // useAccel: 是否使用KD-Tree加速
        RayCastRenderer(SharedScene spScene, bool useAccel = true)
            : spScene               (spScene)
            , scene                 (*spScene)
            , camera                (spScene->camera)
            , useAcceleration       (useAccel)  // 初始化加速标志
        {}

        // 析构函数
        ~RayCastRenderer() = default;

        // 渲染结果类型：像素数据、宽度、高度
        using RenderResult = tuple<RGBA*, unsigned int, unsigned int>;

        // 渲染场景
        // 返回渲染结果
        RenderResult render();

        // 释放渲染结果
        // r: 要释放的渲染结果
        void release(const RenderResult& r);

    private:
        // 伽马校正
        // rgb: 输入的RGB颜色
        // 返回校正后的颜色
        RGB gamma(const RGB& rgb);

        // 光线追踪
        // r: 输入光线
        // 返回该光线对应的颜色
        RGB trace(const Ray& r);

        // 计算最近的相交物体
        // r: 输入光线
        // 返回最近的相交记录
        HitRecord closestHit(const Ray& r);
        
        // 打印性能统计报告
        // renderTime: 渲染时间
        // totalTime: 总时间
        void printPerformanceReport(
            const std::chrono::milliseconds& renderTime, 
            const std::chrono::milliseconds& totalTime);
    };
}

#endif