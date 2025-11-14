// 光线投射渲染器实现
// 实现了基本的光线投射渲染算法，包括阴影计算
#include "RayCastRenderer.hpp"
#include "VertexTransformer.hpp"
#include "intersections/intersections.hpp"
#include <chrono>  // 添加时间库用于性能统计
#include <iostream> // 添加iostream用于输出

namespace RayCast
{
    // 释放渲染结果
    // r: 渲染结果
    void RayCastRenderer::release(const RenderResult& r) {
        auto [p, w, h] = r;
        delete[] p;
    }

    // 伽马校正
    // rgb: 输入的RGB颜色
    // 返回校正后的颜色
    RGB RayCastRenderer::gamma(const RGB& rgb) {
        return glm::sqrt(rgb);
    }

    // 渲染场景
    // 返回渲染结果
    auto RayCastRenderer::render() -> RenderResult {
        auto width = scene.renderOption.width;
        auto height = scene.renderOption.height;
        // 创建像素缓冲区
        auto pixels = new RGBA[width*height];

        // 执行顶点变换
        VertexTransformer vertexTransformer{};
        vertexTransformer.exec(spScene);

        // 性能测量开始
        auto totalStart = std::chrono::high_resolution_clock::now();
        auto buildStart = totalStart;
        
        // 构建KD-Tree（如果启用加速）
        if (useAcceleration) {
            kdTree = std::make_unique<KDTree>(spScene);
            kdTree->build();
            auto buildEnd = std::chrono::high_resolution_clock::now();
            auto buildTime = std::chrono::duration_cast<std::chrono::milliseconds>(buildEnd - buildStart);
            
            std::cout << "=== KD-Tree 构建统计 ===" << std::endl;
            std::cout << "构建时间: " << buildTime.count() << "ms" << std::endl;
            std::cout << "总节点数: " << kdTree->getNodeCount() << std::endl;
            std::cout << "叶节点数: " << kdTree->getLeafCount() << std::endl;
        }

        // 创建着色器程序
        ShaderCreator shaderCreator{};
        for (auto& mtl : scene.materials) {
            shaderPrograms.push_back(shaderCreator.create(mtl, scene.textures));
        }

        // 重置统计（如果使用KD-Tree）
        if (useAcceleration && kdTree) {
            kdTree->resetStats();
        }

        // 对每个像素进行光线追踪
        auto renderStart = std::chrono::high_resolution_clock::now();
        
        for (int i=0; i<height; i++) {
            for (int j=0; j < width; j++) {
                // 生成光线
                auto ray = camera.shoot(float(j)/float(width), float(i)/float(height));
                // 追踪光线获取颜色
                auto color = trace(ray);
                // 颜色后处理
                color = clamp(color);
                color = gamma(color);
                pixels[(height-i-1)*width+j] = {color, 1};
            }
        }
        
        auto renderEnd = std::chrono::high_resolution_clock::now();
        auto totalEnd = renderEnd;
        
        auto renderTime = std::chrono::duration_cast<std::chrono::milliseconds>(renderEnd - renderStart);
        auto totalTime = std::chrono::duration_cast<std::chrono::milliseconds>(totalEnd - totalStart);
        
        // 输出性能报告
        printPerformanceReport(renderTime, totalTime);

        return {pixels, width, height};
    }
    
    // 光线追踪
    // r: 输入光线
    // 返回该光线对应的颜色
    RGB RayCastRenderer::trace(const Ray& r) {
        // 如果场景中没有点光源，返回黑色
        if (scene.pointLightBuffer.size() < 1) return {0, 0, 0};
        // 获取第一个点光源
        auto& l = scene.pointLightBuffer[0];
        // 计算最近的相交物体
        auto closestHitObj = closestHit(r);
        
        if (closestHitObj) {
            auto& hitRec = *closestHitObj;
            // 计算光源方向
            auto out = glm::normalize(l.position - hitRec.hitPoint);
            // 检查光源是否在表面背面
            if (glm::dot(out, hitRec.normal) < 0) {
                return {0, 0, 0};
            }
            // 计算到光源的距离
            auto distance = glm::length(l.position - hitRec.hitPoint);
            // 生成阴影射线
            auto shadowRay = Ray{hitRec.hitPoint, out};
            auto shadowHit = closestHit(shadowRay);
            // 计算着色结果
            auto c = shaderPrograms[hitRec.material.index()]->shade(-r.direction, out, hitRec.normal);
            // 如果没有遮挡或遮挡物在光源后面，返回着色结果
            if ((!shadowHit) || (shadowHit && shadowHit->t > distance)) {
                return c * l.intensity;
            }
            // 否则在阴影中，返回黑色
            else {
                return Vec3{0};
            }
        }
        else {
            return {0, 0, 0};
        }
    }

    // 计算最近的相交物体
    // r: 输入光线
    // 返回最近的相交记录
    HitRecord RayCastRenderer::closestHit(const Ray& r) {
        if (useAcceleration && kdTree) {
            // 使用KD-Tree加速查询
            return kdTree->closestHit(r);
        } else {
            // 使用原来的暴力方法
            HitRecord closestHit = nullopt;
            float closest = FLOAT_INF;
            // 检查与球体的相交
            for (auto& s : scene.sphereBuffer) {
                auto hitRecord = Intersection::xSphere(r, s, 0.01, closest);
                if (hitRecord && hitRecord->t < closest) {
                    closest = hitRecord->t;
                    closestHit = hitRecord;
                }
            }
            // 检查与三角形的相交
            for (auto& t : scene.triangleBuffer) {
                auto hitRecord = Intersection::xTriangle(r, t, 0.01, closest);
                if (hitRecord && hitRecord->t < closest) {
                    closest = hitRecord->t;
                    closestHit = hitRecord;
                }
            }
            // 检查与平面的相交
            for (auto& p : scene.planeBuffer) {
                auto hitRecord = Intersection::xPlane(r, p, 0.01, closest);
                if (hitRecord && hitRecord->t < closest) {
                    closest = hitRecord->t;
                    closestHit = hitRecord;
                }
            }
            return closestHit; 
        }
    }
    
    // 打印性能统计报告
    // renderTime: 渲染时间
    // totalTime: 总时间
    void RayCastRenderer::printPerformanceReport(
        const std::chrono::milliseconds& renderTime, 
        const std::chrono::milliseconds& totalTime) {
        
        std::cout << "\n=== 渲染性能统计 ===" << std::endl;
        std::cout << "渲染时间: " << renderTime.count() << "ms" << std::endl;
        std::cout << "总时间: " << totalTime.count() << "ms" << std::endl;
        
        if (useAcceleration && kdTree) {
            auto& stats = kdTree->getStats();
            
            std::cout << "\n=== KD-Tree 详细统计 ===" << std::endl;
            std::cout << "处理的光线数量: " << stats.raysProcessed << std::endl;
            std::cout << "节点访问总次数: " << stats.totalNodesVisited << std::endl;
            std::cout << "叶节点访问次数: " << stats.totalLeafNodesVisited << std::endl;
            std::cout << "光线-AABB检测次数: " << stats.totalRayAABBTests << std::endl;
            std::cout << "提前退出次数: " << stats.totalEarlyOuts << std::endl;
            
            std::cout << "\n=== 物体检测统计 ===" << std::endl;
            std::cout << "光线-三角形检测: " << stats.totalRayTriangleTests << std::endl;
            std::cout << "光线-球体检测: " << stats.totalRaySphereTests << std::endl;
            std::cout << "光线-平面检测: " << stats.totalRayPlaneTests << std::endl;
            std::cout << "总物体检测次数: " << stats.totalObjectTests() << std::endl;
            
            // 计算效率指标
            int totalObjects = scene.triangleBuffer.size() + scene.sphereBuffer.size() + scene.planeBuffer.size();
            int totalPossibleTests = stats.raysProcessed * totalObjects;
            
            if (totalPossibleTests > 0) {
                double efficiencyRatio = (double)stats.totalObjectTests() / totalPossibleTests;
                double reductionPercent = (1.0 - efficiencyRatio) * 100.0;
                double speedupFactor = (double)totalPossibleTests / stats.totalObjectTests();
                
                std::cout << "\n=== 效率分析 ===" << std::endl;
                std::cout << "场景物体总数: " << totalObjects << std::endl;
                std::cout << "暴力方法检测次数: " << totalPossibleTests << std::endl;
                std::cout << "KD-Tree检测次数: " << stats.totalObjectTests() << std::endl;
                std::cout << "检测减少比例: " << reductionPercent << "%" << std::endl;
                std::cout << "加速倍数: " << speedupFactor << "x" << std::endl;
            }
            
            // 平均每次光线查询的统计
            if (stats.raysProcessed > 0) {
                std::cout << "\n=== 每光线平均统计 ===" << std::endl;
                std::cout << "平均节点访问: " << (double)stats.totalNodesVisited / stats.raysProcessed << std::endl;
                std::cout << "平均物体检测: " << (double)stats.totalObjectTests() / stats.raysProcessed << std::endl;
                std::cout << "平均AABB检测: " << (double)stats.totalRayAABBTests / stats.raysProcessed << std::endl;
            }
        } else {
            // 暴力方法的统计
            std::cout << "\n=== 暴力方法统计 ===" << std::endl;
            int totalObjects = scene.triangleBuffer.size() + scene.sphereBuffer.size() + scene.planeBuffer.size();
            int totalRays = scene.renderOption.width * scene.renderOption.height;
            std::cout << "总光线数量: " << totalRays << std::endl;
            std::cout << "场景物体总数: " << totalObjects << std::endl;
            std::cout << "总检测次数: " << totalRays * totalObjects << std::endl;
        }
    }
}