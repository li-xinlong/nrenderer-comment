#include "shaders/Glass.hpp"
#include <cmath>

namespace RayCast
{
    // 构造函数
    // material: 材质参数
    // textures: 纹理数组
    Glass::Glass(Material &material, vector<Texture> &textures)
        : Shader(material, textures)
    {
        using PW = Property::Wrapper;

        // 获取折射率
        auto optIOR = material.getProperty<PW::FloatType>("ior");
        if (optIOR)
            ior = (*optIOR).value;
        else
            ior = 1.5f; // 默认为1.5（玻璃的典型折射率）

        // 获取镜面反射颜色
        auto optSpecularColor = material.getProperty<PW::RGBType>("specularColor");
        if (optSpecularColor)
            specularColor = (*optSpecularColor).value;
        else
            specularColor = {1.0f, 1.0f, 1.0f}; // 默认为白色

        std::cout << "创建Glass材质: ior=" << ior
                  << ", specularColor=(" << specularColor.r << ", "
                  << specularColor.g << ", " << specularColor.b << ")" << std::endl;
    }

    // 计算着色结果
    // in: 入射方向
    // out: 出射方向
    // normal: 表面法线
    // 返回着色计算的颜色值
    RGB Glass::shade(const Vec3 &in, const Vec3 &out, const Vec3 &normal) const
    {
        // 对于玻璃材质，我们主要计算反射和折射
        // 但在当前的光子映射框架中，我们简化处理，只返回镜面反射颜色

        // 计算反射方向
        Vec3 reflectDir = reflect(normal, -in);

        // 计算菲涅尔反射率
        float fresnelReflectance = fresnel(normal, -in, ior);

        // 玻璃的颜色主要由菲涅尔效应决定
        // 我们使用镜面反射颜色乘以菲涅尔反射率
        return specularColor * fresnelReflectance;
    }

    // 计算反射方向
    Vec3 Glass::reflect(const Vec3 &normal, const Vec3 &direction)
    {
        return direction - 2.0f * glm::dot(direction, normal) * normal;
    }

    // 计算折射方向
    Vec3 Glass::refract(const Vec3 &normal, const Vec3 &direction, float ior, bool &totalInternalReflection)
    {
        Vec3 norm = normal;
        float cosi = glm::clamp(glm::dot(direction, norm), -1.0f, 1.0f);
        float etai = 1.0f, etat = ior;

        // 判断光线是从外部进入还是内部射出
        if (cosi < 0.0f)
        {
            // 从外部进入
            cosi = -cosi;
        }
        else
        {
            // 从内部射出，交换折射率并反转法线
            std::swap(etai, etat);
            norm = -norm;
        }

        float eta = etai / etat;
        float k = 1.0f - eta * eta * (1.0f - cosi * cosi);

        if (k < 0.0f)
        {
            // 全内反射
            totalInternalReflection = true;
            return reflect(norm, direction);
        }
        else
        {
            // 正常折射
            totalInternalReflection = false;
            return eta * direction + (eta * cosi - std::sqrt(k)) * norm;
        }
    }

    // 计算菲涅尔反射率
    float Glass::fresnel(const Vec3 &normal, const Vec3 &direction, float ior)
    {
        Vec3 norm = normal;
        float cosi = glm::clamp(glm::dot(direction, norm), -1.0f, 1.0f);
        float etai = 1.0f, etat = ior;

        if (cosi > 0.0f)
        {
            std::swap(etai, etat);
        }

        // 计算sint
        float sint = etai / etat * std::sqrt(std::max(0.0f, 1.0f - cosi * cosi));

        if (sint >= 1.0f)
        {
            // 全内反射
            return 1.0f;
        }
        else
        {
            float cost = std::sqrt(std::max(0.0f, 1.0f - sint * sint));
            cosi = std::abs(cosi);

            // 计算平行和垂直偏振的反射率
            float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
            float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));

            // 返回非偏振光的平均反射率
            return (Rs * Rs + Rp * Rp) / 2.0f;
        }
    }
}