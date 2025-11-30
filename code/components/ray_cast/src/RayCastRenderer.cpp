#include "RayCastRenderer.hpp"
#include "VertexTransformer.hpp"
#include "intersections/intersections.hpp"
#include <random>
#include <iostream>
#include <chrono>
#include <algorithm>
#include <unordered_set>

namespace RayCast
{
	void RayCastRenderer::release(const RenderResult &r)
	{
		auto [p, w, h] = r;
		delete[] p;
	}

	RGB RayCastRenderer::gamma(const RGB &rgb)
	{
		return glm::sqrt(rgb);
	}

	// bool RayCastRenderer::russianRoulette(float terminationProbability)
	// {
	// 	static std::random_device rd;
	// 	static std::mt19937 gen(rd());
	// 	std::uniform_real_distribution<> dis(0.0, 1.0);

	// 	float randVal = dis(gen);
	// 	bool terminated = randVal < terminationProbability;

	// 	// 调试：检查俄罗斯轮盘赌的实际工作
	// 	static int totalCalls = 0;
	// 	static int terminationCount = 0;
	// 	totalCalls++;

	// 	if (terminated)
	// 	{
	// 		terminationCount++;
	// 	}

	// 	if (totalCalls < 50)
	// 	{
	// 		std::cout << "俄罗斯轮盘赌: 概率=" << terminationProbability
	// 				  << ", 随机值=" << randVal
	// 				  << ", 终止=" << terminated << std::endl;
	// 	}

	// 	if (totalCalls == 50)
	// 	{
	// 		std::cout << "俄罗斯轮盘赌统计: 调用次数=" << totalCalls
	// 				  << ", 终止次数=" << terminationCount
	// 				  << ", 实际终止率=" << (float(terminationCount) / totalCalls * 100.0f) << "%" << std::endl;
	// 	}

	// 	return terminated;
	// }
	bool RayCastRenderer::russianRoulette(int bounce)
	{
		float terminationProbability;
		if (bounce == 0)
		{
			terminationProbability = 0.0f; // 必须反弹
		}
		else if (bounce == 1)
		{
			terminationProbability = 0.2f; // 仅 20% 终止（让更多光子完成 bounce=2）
		}
		else
		{
			terminationProbability = 0.8f; // 高阶快速终止
		}

		static std::random_device rd;
		static std::mt19937 gen(rd());
		std::uniform_real_distribution<> dis(0.0, 1.0);
		return dis(gen) < terminationProbability;
	}

	// 生成半球随机方向（均匀采样）
	Vec3 RayCastRenderer::randomHemisphereDirection(const Vec3 &normal)
	{
		static std::random_device rd;
		static std::mt19937 gen(rd());
		std::uniform_real_distribution<> dis(0.0, 1.0);

		float u1 = dis(gen);
		float u2 = dis(gen);

		float theta = 2.0f * PI * u1;
		float phi = std::acos(1.0f - 2.0f * u2);

		Vec3 localDir;
		localDir.x = std::sin(phi) * std::cos(theta);
		localDir.y = std::sin(phi) * std::sin(theta);
		localDir.z = std::cos(phi);

		// 构建局部坐标系
		Vec3 tangent, bitangent;
		if (std::abs(normal.x) > std::abs(normal.y))
		{
			tangent = Vec3(normal.z, 0, -normal.x);
		}
		else
		{
			tangent = Vec3(0, -normal.z, normal.y);
		}
		tangent = glm::normalize(tangent);
		bitangent = glm::cross(normal, tangent);

		// 转换到世界坐标系
		return localDir.x * tangent + localDir.y * bitangent + localDir.z * normal;
	}

	// 生成余弦加权随机方向
	Vec3 RayCastRenderer::randomCosineWeightedDirection(const Vec3 &normal)
	{
		static std::random_device rd;
		static std::mt19937 gen(rd());
		std::uniform_real_distribution<> dis(0.0, 1.0);

		float u1 = dis(gen);
		float u2 = dis(gen);

		float r = std::sqrt(u1);
		float theta = 2.0f * PI * u2;

		float x = r * std::cos(theta);
		float y = r * std::sin(theta);
		float z = std::sqrt(std::max(0.0f, 1.0f - u1));

		Vec3 localDir(x, y, z);

		// 构建局部坐标系
		Vec3 tangent, bitangent;
		if (std::abs(normal.x) > std::abs(normal.y))
		{
			tangent = Vec3(normal.z, 0, -normal.x);
		}
		else
		{
			tangent = Vec3(0, -normal.z, normal.y);
		}
		tangent = glm::normalize(tangent);
		bitangent = glm::cross(normal, tangent);

		// 转换到世界坐标系
		return localDir.x * tangent + localDir.y * bitangent + localDir.z * normal;
	}

	// 获取材质反射率
	RGB RayCastRenderer::getMaterialReflectance(int materialIndex) const
	{
		if (materialIndex < 0 || materialIndex >= scene.materials.size())
		{
			return RGB(0.7f, 0.7f, 0.7f);
		}

		auto &material = scene.materials[materialIndex];

		// 调试输出
		static int callCount = 0;
		if (callCount < 20)
		{
			std::cout << "获取材质反射率: 索引=" << materialIndex
					  << ", 类型=" << material.type << std::endl;
			callCount++;
		}

		if (material.type == 0 || material.type == 1)
		{ // Lambertian 或 Phong
			auto optDiffuseColor = material.getProperty<Property::Wrapper::RGBType>("diffuseColor");
			if (optDiffuseColor)
			{
				RGB color = (*optDiffuseColor).value;
				if (callCount < 20)
				{
					std::cout << "  漫反射颜色: (" << color.r << ", " << color.g << ", " << color.b << ")" << std::endl;
				}
				return color;
			}
		}

		return RGB(0.7f, 0.7f, 0.7f);
	}

	// 场景分析
	void RayCastRenderer::analyzeScene()
	{
		std::cout << "=== 场景分析 ===" << std::endl;
		std::cout << "材质数量: " << scene.materials.size() << std::endl;

		int lambertCount = 0;
		int phongCount = 0;

		for (size_t i = 0; i < scene.materials.size(); ++i)
		{
			auto &material = scene.materials[i];
			std::cout << "材质 " << i << ": 类型=" << material.type;

			auto optDiffuse = material.getProperty<Property::Wrapper::RGBType>("diffuseColor");
			if (optDiffuse)
			{
				std::cout << ", 漫反射颜色=(" << (*optDiffuse).value.r << ", "
						  << (*optDiffuse).value.g << ", " << (*optDiffuse).value.b << ")";
			}

			if (material.type == 0)
			{
				std::cout << " (Lambertian)";
				lambertCount++;
			}
			else if (material.type == 1)
			{
				std::cout << " (Phong)";
				phongCount++;
			}

			std::cout << std::endl;
		}

		std::cout << "几何体统计:" << std::endl;
		std::cout << "  三角形数量: " << scene.triangleBuffer.size() << std::endl;
		std::cout << "  球体数量: " << scene.sphereBuffer.size() << std::endl;
		std::cout << "  平面数量: " << scene.planeBuffer.size() << std::endl;

		std::cout << "光源统计:" << std::endl;
		std::cout << "  点光源数量: " << scene.pointLightBuffer.size() << std::endl;
		if (!scene.pointLightBuffer.empty())
		{
			auto &light = scene.pointLightBuffer[0];
			std::cout << "  主光源位置: (" << light.position.x << ", " << light.position.y << ", " << light.position.z << ")" << std::endl;
			std::cout << "  主光源强度: (" << light.intensity.r << ", " << light.intensity.g << ", " << light.intensity.b << ")" << std::endl;
		}

		std::cout << "材质类型统计:" << std::endl;
		std::cout << "  Lambertian材质: " << lambertCount << std::endl;
		std::cout << "  Phong材质: " << phongCount << std::endl;

		if (lambertCount == 0)
		{
			std::cout << "? 警告: 场景中没有Lambertian材质，光子映射可能无法正常工作" << std::endl;
		}
	}

	void RayCastRenderer::buildPhotonMap()
	{
		globalPhotonMap.clear();
		nextPhotonId = 0;

		if (scene.pointLightBuffer.empty())
		{
			std::cout << "警告: 场景中没有点光源" << std::endl;
			return;
		}

		auto &light = scene.pointLightBuffer[0];

		std::cout << "开始构建光子图，发射 " << photonCount << " 个光子..." << std::endl;
		auto startTime = std::chrono::steady_clock::now();

		int storedPhotons = 0;
		int emittedPhotons = 0;

		// 用于跟踪每个光子ID的存储状态
		std::unordered_set<int> storedPhotonIds;

		// 计算每个光子的初始能量 - 修正能量计算
		// 光源强度是辐射强度（W/sr），需要转换为每个光子的能量
		// 使用更合理的能量分配
		// RGB initialPower = light.intensity / float(photonCount);
		RGB initialPower = light.intensity * (2.0f * PI) / float(photonCount);
		for (int i = 0; i < photonCount; ++i)
		{
			emittedPhotons++;
			int photonId = nextPhotonId++;

			if (photonCount >= 1000 && i % 1000 == 0)
			{
				auto currentTime = std::chrono::steady_clock::now();
				auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(currentTime - startTime).count();
				std::cout << "进度: " << i << "/" << photonCount
						  << " (" << (i * 100 / photonCount) << "%), "
						  << "已用时间: " << elapsed << "秒" << std::endl;
			}

			// 从光源向随机方向发射光子
			static std::random_device rd;
			static std::mt19937 gen(rd());
			std::uniform_real_distribution<> dis(0.0, 1.0);
			float phi = 2.0f * PI * dis(gen);
			float theta = std::acos(1.0f - 2.0f * dis(gen));
			theta = theta * 0.5f;
			Vec3 direction;
			direction.x = std::sin(theta) * std::cos(phi);
			direction.y = -std::abs(std::sin(theta) * std::sin(phi)); // 主要向下发射
			direction.z = std::cos(theta);
			direction = glm::normalize(direction);
			Ray ray(light.position + direction * 0.001f, direction);

			// 使用修正后的初始能量
			RGB power = initialPower;

			tracePhoton(ray, power, 0, storedPhotons, photonId, storedPhotonIds);
		}

		// 构建KD树加速结构
		std::cout << "构建KD树加速结构..." << std::endl;
		auto kdTreeStart = std::chrono::steady_clock::now();
		globalPhotonMap.buildKDTree();
		auto kdTreeEnd = std::chrono::steady_clock::now();
		auto kdTreeTime = std::chrono::duration_cast<std::chrono::milliseconds>(kdTreeEnd - kdTreeStart).count();

		auto endTime = std::chrono::steady_clock::now();
		auto totalTime = std::chrono::duration_cast<std::chrono::seconds>(endTime - startTime).count();

		std::cout << "光子图构建完成，耗时 " << totalTime << " 秒" << std::endl;
		std::cout << "KD树构建耗时: " << kdTreeTime << " 毫秒" << std::endl;
		std::cout << "发射光子数量: " << emittedPhotons << std::endl;
		std::cout << "存储光子数量: " << storedPhotons << std::endl;
		std::cout << "存储率: " << (float(storedPhotons) / emittedPhotons * 100.0f) << "%" << std::endl;

		// 验证存储光子数不超过发射光子数
		if (storedPhotons > emittedPhotons)
		{
			std::cout << "严重错误: 存储光子数超过发射光子数" << std::endl;
		}
		else if (storedPhotons == 0)
		{
			std::cout << "警告: 没有存储任何光子" << std::endl;
		}
		else
		{
			std::cout << "光子图构建成功" << std::endl;
		}
	}
	bool RayCastRenderer::isLambertianMaterial(int materialIndex) const
	{
		if (materialIndex < 0 || materialIndex >= scene.materials.size())
		{
			return false;
		}

		auto &material = scene.materials[materialIndex];
		return material.type == 0; // 0表示Lambertian材质
	}

	bool RayCastRenderer::isDielectricMaterial(int materialIndex) const
	{
		if (materialIndex < 0 || materialIndex >= scene.materials.size())
		{
			return false;
		}

		auto &material = scene.materials[materialIndex];
		return material.type == 2; // 2表示Glass材质
	}

	void RayCastRenderer::tracePhoton(const Ray &ray, RGB power, int bounce, int &storedCount,
									  int photonId, std::unordered_set<int> &storedPhotonIds)
	{
		static int totalTraces = 0;
		totalTraces++;

		// 最大反弹限制
		if (bounce >= maxBounces)
		{
			return;
		}

		// 能量过低，提前终止
		float currentEnergy = power.r + power.g + power.b;
		if (currentEnergy < 1e-6f)
		{
			return;
		}

		// 求交
		auto hitRecord = closestHit(ray);
		if (!hitRecord)
		{
			return; // 光子逃逸场景
		}

		auto &rec = *hitRecord;
		int materialIndex = rec.material.index();

		if (materialIndex < 0 || materialIndex >= static_cast<int>(scene.materials.size()))
		{
			return;
		}

		auto &material = scene.materials[materialIndex];

		// 调试输出（前20次）
		if (totalTraces <= 20)
		{
			std::cout << "光子追踪 #" << totalTraces
					  << ", 光子ID: " << photonId
					  << ", 反弹: " << bounce
					  << ", 材质类型: " << material.type
					  << ", 当前能量: " << currentEnergy << std::endl;
		}

		// ========================
		// Glass / Dielectric 材质（不存储光子）
		// ========================
		if (isDielectricMaterial(materialIndex))
		{
			float materialIOR = 1.5f;
			auto optIOR = material.getProperty<Property::Wrapper::FloatType>("ior");
			if (optIOR)
				materialIOR = (*optIOR).value;

			float fresnelReflectance = Glass::fresnel(rec.normal, ray.direction, materialIOR);

			// 反射分支
			RGB reflectPower = power * fresnelReflectance;
			if ((reflectPower.r + reflectPower.g + reflectPower.b) > 1e-6f)
			{
				Vec3 reflectDir = Glass::reflect(rec.normal, ray.direction);
				Ray reflectRay(rec.hitPoint + rec.normal * 0.001f, reflectDir);
				tracePhoton(reflectRay, reflectPower, bounce + 1, storedCount, photonId, storedPhotonIds);
			}

			// 折射分支
			bool totalInternalReflection = false;
			Vec3 refractDir = Glass::refract(rec.normal, ray.direction, materialIOR, totalInternalReflection);
			if (!totalInternalReflection)
			{
				RGB refractPower = power * (1.0f - fresnelReflectance);
				if ((refractPower.r + refractPower.g + refractPower.b) > 1e-6f)
				{
					Ray refractRay(rec.hitPoint - rec.normal * 0.001f, refractDir);
					tracePhoton(refractRay, refractPower, bounce + 1, storedCount, photonId, storedPhotonIds);
				}
			}

			return;
		}

		// ========================
		// Lambertian 材质处理（唯一存储光子的材质）
		// ========================
		if (isLambertianMaterial(materialIndex))
		{
			// === 【关键修复】概率性存储光子 ===
			if (bounce >= 1)
			{
				if (currentEnergy > 1e-5f && storedPhotonIds.find(photonId) == storedPhotonIds.end())
				{
					// 存储概率：能量越高，越可能被记录
					const float scale = 0.2f; // 可调：0.1~0.3
					float storeProbability = std::min(1.0f, currentEnergy * scale / 0.001f);

					static std::random_device rd_store;
					static std::mt19937 gen_store(rd_store());
					std::uniform_real_distribution<> dis_store(0.0, 1.0);

					if (dis_store(gen_store) < storeProbability)
					{
						Photon photon(rec.hitPoint, -ray.direction, power, bounce, photonId);
						globalPhotonMap.store(photon);
						storedPhotonIds.insert(photonId);
						storedCount++;

						static int debugStoreCount = 0;
						if (debugStoreCount < 10 && totalTraces <= 100)
						{
							std::cout << "存储光子 #" << debugStoreCount
									  << " - ID: " << photonId
									  << ", bounce: " << bounce
									  << ", energy: " << currentEnergy
									  << ", 概率: " << storeProbability << std::endl;
							debugStoreCount++;
						}
					}
				}
			}

			// === Russian Roulette 决定是否继续传播 ===
			if (russianRoulette(bounce))
			{
				if (totalTraces <= 20)
				{
					std::cout << "  Lambertian - 俄罗斯轮盘赌终止 (bounce=" << bounce << ")" << std::endl;
				}
				return;
			}

			// === 生成新方向并继续追踪 ===
			Vec3 newDirection = randomCosineWeightedDirection(rec.normal);
			float cosTheta = glm::dot(newDirection, rec.normal);
			if (cosTheta <= 0.0f)
				return;

			RGB albedo = getMaterialReflectance(materialIndex);
			RGB brdf = albedo / PI;
			RGB newPower = power * brdf * cosTheta;

			if ((newPower.r + newPower.g + newPower.b) < 1e-6f)
			{
				return;
			}

			Ray newRay(rec.hitPoint + rec.normal * 0.001f, glm::normalize(newDirection));
			tracePhoton(newRay, newPower, bounce + 1, storedCount, photonId, storedPhotonIds);
			return;
		}

		// ========================
		// 其他材质（如 Phong、Metal 等）：不存储，高概率终止
		// ========================
		float terminationProbOther = (bounce == 0) ? 0.3f : 0.9f;
		static std::random_device rd_other;
		static std::mt19937 gen_other(rd_other());
		std::uniform_real_distribution<> dis_other(0.0, 1.0);

		if (dis_other(gen_other) < terminationProbOther)
		{
			if (totalTraces <= 20)
			{
				std::cout << "  其他材质 - 高概率终止" << std::endl;
			}
			return;
		}

		// 若未终止，简单反射或终止（此处简化为终止）
		return;
	}

	// 光子图方法实现
	RGB PhotonMap::estimateRadiance(const Vec3 &position, const Vec3 &normal, float radius) const
	{
		if (photons.empty())
			return RGB(0);

		// 使用KD树进行范围查询
		auto nearbyPhotons = queryRange(position, radius);

		if (nearbyPhotons.empty())
			return RGB(0);

		// 实验四公式: L_r ≈ Σ f_r(x, Ψ_r, Ψ_{i,p}) * ΔΦ_p(x, Ψ_{i,p}) / (π * r?)
		RGB totalRadiance(0);
		int validCount = 0;

		for (const auto &photon : nearbyPhotons)
		{
			// 只考虑与表面法线方向一致的光子
			if (glm::dot(photon.direction, normal) > 0.1f)
			{
				// 对于Lambertian材质: f_r = albedo / π
				RGB albedo(0.7f, 0.7f, 0.7f); // 默认值
				RGB brdf = albedo / PI;

				// 光子通量 ΔΦ_p
				RGB photonFlux = photon.power;

				// 累加贡献: f_r * ΔΦ_p
				totalRadiance += brdf * photonFlux;
				validCount++;
			}
		}

		if (validCount == 0)
			return RGB(0);

		// 除以区域面积: π * r?
		float area = PI * radius * radius;
		RGB radiance = totalRadiance / area;

		return clamp(radiance);
	}

	// 自适应半径的光子密度估计
	RGB PhotonMap::estimateRadianceAdaptive(const Vec3 &position, const Vec3 &normal, int nearestPhotons) const
	{
		if (photons.empty() || nearestPhotons <= 0)
			return RGB(0);

		// 找到最近的N个光子
		std::vector<std::pair<float, const Photon *>> photonDistances;

		for (const auto &photon : photons)
		{
			float distance = glm::length(photon.position - position);
			photonDistances.emplace_back(distance, &photon);
		}

		// 按距离排序
		std::sort(photonDistances.begin(), photonDistances.end(),
				  [](const auto &a, const auto &b)
				  { return a.first < b.first; });

		// 确保有足够的光子
		if (photonDistances.size() < nearestPhotons)
		{
			nearestPhotons = photonDistances.size();
		}

		if (nearestPhotons == 0)
			return RGB(0);

		// 使用第K个光子的距离作为半径（K=nearestPhotons-1）
		float radius = photonDistances[nearestPhotons - 1].first;

		// 确保最小半径
		if (radius < 10.0f) // 增加最小半径
			radius = 10.0f;

		// 计算辐射亮度
		RGB totalRadiance(0);
		int validPhotons = 0;

		for (int i = 0; i < nearestPhotons; ++i)
		{
			const Photon &photon = *photonDistances[i].second;

			// 放宽法线方向限制
			if (glm::dot(photon.direction, normal) > -0.5f) // 允许更多光子参与
			{
				// 使用实际材质的反射率，这里暂时用默认值
				RGB albedo(0.7f, 0.7f, 0.7f);
				RGB brdf = albedo / PI;

				totalRadiance += brdf * photon.power;
				validPhotons++;
			}
		}

		if (validPhotons > 0)
		{
			float area = PI * radius * radius;
			RGB radiance = totalRadiance / area;

			// 增强间接光照效果
			return radiance * 10000.0f;
		}

		return RGB(0);
	}
	// 光子密度可视化
	RGB PhotonMap::visualizePhotonDensity() const
	{
		if (photons.empty())
			return RGB(0);

		float density = std::min(1.0f, photons.size() / 10000.0f);
		return RGB(density, density * 0.8f, density * 0.5f);
	}

	RGB PhotonMap::getTotalEnergy() const
	{
		RGB total(0);
		for (const auto &photon : photons)
		{
			total += photon.power;
		}
		return total;
	}

	// 光子可视化渲染
	RGB RayCastRenderer::renderPhotonVisualization()
	{
		if (globalPhotonMap.size() == 0)
			return RGB(0, 0, 0);

		float intensity = std::min(1.0f, globalPhotonMap.size() / 10000.0f);
		return RGB(intensity, intensity, intensity);
	}

	auto RayCastRenderer::render() -> RenderResult
	{
		auto width = scene.renderOption.width;
		auto height = scene.renderOption.height;
		auto pixels = new RGBA[width * height];

		// 执行顶点变换
		VertexTransformer vertexTransformer{};
		vertexTransformer.exec(spScene);

		// 分析场景
		analyzeScene();

		// 构建光子图
		if (usePhotonMapping)
		{
			buildPhotonMap();
		}
		else
		{
			std::cout << "使用传统光线追踪模式" << std::endl;
		}

		// 创建着色器程序
		ShaderCreator shaderCreator{};
		shaderPrograms.clear();
		for (auto &mtl : scene.materials)
		{
			shaderPrograms.push_back(shaderCreator.create(mtl, scene.textures));
		}

		std::cout << "开始渲染场景..." << std::endl;
		auto renderStart = std::chrono::steady_clock::now();

		// 对每个像素进行光线追踪
		for (int i = 0; i < height; i++)
		{
			for (int j = 0; j < width; j++)
			{
				RGB finalColor(0, 0, 0);

				if (usePhotonMapping && globalPhotonMap.size() > 0)
				{
					// 光子映射模式
					auto ray = camera.shoot(float(j) / float(width), float(i) / float(height));
					auto hitRecord = closestHit(ray);

					if (hitRecord)
					{
						auto &rec = *hitRecord;

						// 调试：输出光子图信息
						if (i == height / 2 && j == width / 2)
						{
							std::cout << "中心像素光子图查询:" << std::endl;
							std::cout << "  命中点: (" << rec.hitPoint.x << ", "
									  << rec.hitPoint.y << ", " << rec.hitPoint.z << ")" << std::endl;
							std::cout << "  法线: (" << rec.normal.x << ", "
									  << rec.normal.y << ", " << rec.normal.z << ")" << std::endl;

							// 测试光子图查询
							auto nearbyPhotons = globalPhotonMap.queryRange(rec.hitPoint, 50.0f);
							std::cout << "  附近光子数: " << nearbyPhotons.size() << std::endl;

							if (!nearbyPhotons.empty())
							{
								std::cout << "  第一个光子能量: (" << nearbyPhotons[0].power.r << ", "
										  << nearbyPhotons[0].power.g << ", " << nearbyPhotons[0].power.b << ")" << std::endl;
							}
						}
						// 使用自适应半径方法计算间接光照
						RGB indirectRadiance = globalPhotonMap.estimateRadianceAdaptive(rec.hitPoint, rec.normal, 50);
						if (i == height / 2 && j == width / 2)
						{
							std::cout << "  估计的间接光照: (" << indirectRadiance.r << ", "
									  << indirectRadiance.g << ", " << indirectRadiance.b << ")" << std::endl;
						}

						// 直接光照
						RGB directRadiance(0);
						if (scene.pointLightBuffer.size() > 0)
						{
							auto &light = scene.pointLightBuffer[0];
							Vec3 lightDir = glm::normalize(light.position - rec.hitPoint);

							// 检查阴影
							Ray shadowRay(rec.hitPoint + rec.normal * 0.001f, lightDir);
							auto shadowHit = closestHit(shadowRay);
							float shadowFactor = 1.0f;
							if (shadowHit && shadowHit->t < glm::length(light.position - rec.hitPoint))
							{
								shadowFactor = 0.3f;
							}

							// 使用着色器计算直接光照
							directRadiance = shaderPrograms[rec.material.index()]->shade(-ray.direction, lightDir, rec.normal);
							directRadiance *= light.intensity * shadowFactor;
						}

						// 最终颜色 = 直接光照 + 间接光照
						finalColor = directRadiance + indirectRadiance * 100.0f;
					}
					else
					{
						finalColor = RGB(0.1f, 0.1f, 0.3f);
					}
				}
				else
				{
					// 传统光线追踪模式
					auto ray = camera.shoot(float(j) / float(width), float(i) / float(height));
					finalColor = trace(ray);
				}

				finalColor = clamp(finalColor);
				finalColor = gamma(finalColor);
				pixels[(height - i - 1) * width + j] = {finalColor, 1};
			}
		}

		auto renderEnd = std::chrono::steady_clock::now();
		auto renderTime = std::chrono::duration_cast<std::chrono::seconds>(renderEnd - renderStart).count();
		std::cout << "场景渲染完成，耗时 " << renderTime << " 秒" << std::endl;

		// 能量守恒验证
		if (usePhotonMapping && globalPhotonMap.size() > 0)
		{
			verifyEnergyConservation();
		}

		return {pixels, width, height};
	}

	// 能量守恒验证函数
	void RayCastRenderer::verifyEnergyConservation()
	{
		if (scene.pointLightBuffer.empty())
			return;

		auto &light = scene.pointLightBuffer[0];

		// 修正光源发射总能量计算
		// 点光源的辐射强度是给定的，总发射能量 = 强度 * 4π（全立体角）
		// 但我们只向半球发射光子，所以使用 2π
		RGB emittedEnergy = light.intensity * (2.0f * PI);

		RGB storedEnergy = globalPhotonMap.getTotalEnergy();

		std::cout << "=== 能量守恒验证 ===" << std::endl;
		std::cout << "光源发射总能量: (" << emittedEnergy.r << ", " << emittedEnergy.g << ", " << emittedEnergy.b << ")" << std::endl;
		std::cout << "光子图存储总能量: (" << storedEnergy.r << ", " << storedEnergy.g << ", " << storedEnergy.b << ")" << std::endl;

		float emittedScalar = emittedEnergy.r + emittedEnergy.g + emittedEnergy.b;
		float storedScalar = storedEnergy.r + storedEnergy.g + storedEnergy.b;

		std::cout << "光源发射标量能量: " << emittedScalar << std::endl;
		std::cout << "光子图存储标量能量: " << storedScalar << std::endl;

		if (emittedScalar > 0)
		{
			float energyRatio = storedScalar / emittedScalar;
			std::cout << "能量保持率: " << (energyRatio * 100.0f) << "%" << std::endl;

			// 修正合理的能量保持率范围
			// 由于光子逃逸、吸收和俄罗斯轮盘赌，保持率应该在30%-70%之间
			if (energyRatio > 0.3f && energyRatio < 0.7f)
			{
				std::cout << "能量守恒验证合理" << std::endl;
			}
			else if (energyRatio < 0.3f)
			{
				std::cout << "能量损失过多: 可能由于光子逃逸或过早终止" << std::endl;
			}
			else
			{
				std::cout << "能量保持异常: 能量保持率过高，可能由于能量计算错误" << std::endl;
				std::cout << "可能原因:" << std::endl;
				std::cout << "  - 光子能量没有正确衰减" << std::endl;
				std::cout << "  - 俄罗斯轮盘赌终止概率设置不当" << std::endl;
				std::cout << "  - 光源能量计算错误" << std::endl;
			}
		}
	}

	RGB RayCastRenderer::trace(const Ray &r)
	{
		auto hitRecord = closestHit(r);
		if (hitRecord)
		{
			auto &rec = *hitRecord;

			if (scene.pointLightBuffer.size() > 0)
			{
				auto &light = scene.pointLightBuffer[0];
				Vec3 out = glm::normalize(light.position - rec.hitPoint);

				if (glm::dot(out, rec.normal) > 0)
				{
					return shaderPrograms[rec.material.index()]->shade(-r.direction, out, rec.normal) * light.intensity;
				}
			}
		}
		return RGB(0, 0, 0);
	}

	HitRecord RayCastRenderer::closestHit(const Ray &r)
	{
		HitRecord closestHit = nullopt;
		float closest = FLOAT_INF;

		for (auto &s : scene.sphereBuffer)
		{
			auto hitRecord = Intersection::xSphere(r, s, 0.01, closest);
			if (hitRecord && hitRecord->t < closest)
			{
				closest = hitRecord->t;
				closestHit = hitRecord;
			}
		}

		for (auto &t : scene.triangleBuffer)
		{
			auto hitRecord = Intersection::xTriangle(r, t, 0.01, closest);
			if (hitRecord && hitRecord->t < closest)
			{
				closest = hitRecord->t;
				closestHit = hitRecord;
			}
		}

		for (auto &p : scene.planeBuffer)
		{
			auto hitRecord = Intersection::xPlane(r, p, 0.01, closest);
			if (hitRecord && hitRecord->t < closest)
			{
				closest = hitRecord->t;
				closestHit = hitRecord;
			}
		}

		return closestHit;
	}
}