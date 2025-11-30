#pragma once
#ifndef __GLASS_HPP__
#define __GLASS_HPP__

#include "Shader.hpp"

namespace RayCast
{
    // Glass材质着色器类
    // 实现了电介质材质（玻璃）的反射和折射计算
    class Glass : public Shader
    {
    private:
        float ior;         // 折射率 (Index of Refraction)
        RGB specularColor; // 镜面反射颜色

    public:
        // 构造函数
        // material: 材质参数
        // textures: 纹理数组
        Glass(Material &material, vector<Texture> &textures);

        // 计算着色结果
        // in: 入射方向
        // out: 出射方向
        // normal: 表面法线
        // 返回着色计算的颜色值
        virtual RGB shade(const Vec3 &in, const Vec3 &out, const Vec3 &normal) const;

        // 计算反射方向
        static Vec3 reflect(const Vec3 &normal, const Vec3 &direction);

        // 计算折射方向
        static Vec3 refract(const Vec3 &normal, const Vec3 &direction, float ior, bool &totalInternalReflection);

        // 计算菲涅尔反射率
        static float fresnel(const Vec3 &normal, const Vec3 &direction, float ior);
    };
}

#endif