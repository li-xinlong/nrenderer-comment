#include "server/Server.hpp"
#include "component/RenderComponent.hpp"
#include "RayCastRenderer.hpp"

using namespace std;
using namespace NRenderer;

namespace RayCast
{
	class Adapter : public RenderComponent
	{
	public:
		void render(SharedScene spScene)
		{
			// 创建光子映射渲染器
			RayCastRenderer renderer{ spScene };

			// 设置光子映射参数
			renderer.setPhotonCount(100);		 // 默认10000个光子
			renderer.setMaxBounces(5);			 // 最大反弹5次
			renderer.setUsePhotonMapping(true); // 启用光子映射

			auto result = renderer.render();

			// 获取渲染结果并设置到屏幕
			auto [pixels, width, height] = result;
			getServer().screen.set(pixels, width, height);

			// 释放渲染结果
			renderer.release(result);
		}
	};
}

const static string description =
"Photon Mapping Renderer with KD-Tree Optimization.\n"
"Features:\n"
" - Photon Mapping Algorithm\n"
" - KD-Tree Accelerated Photon Search\n"
" - Global Photon Map\n"
" - Direct + Indirect Lighting\n"
" - Shadow Calculation\n"
" - Energy Conservation Verification\n"
" - Supports 1,000-100,000 photons\n"
"Please use Cornell Box scene for best results";

REGISTER_RENDERER(RayCast, description, RayCast::Adapter);