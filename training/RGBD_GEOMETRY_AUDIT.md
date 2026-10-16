# RGBD 几何离线核对（2026-09-12）

核对对象：Windows D:/handyman/handyman-unity2/handyman-unity2 项目副本。
此报告不证明另一台 Unity 电脑的当前场景或已构建 Handyman.exe 与副本完全一致。
本次没有启动 Unity、修改比赛场景或解除 geometry_verified 限制。

## 已取得的代码证据

- Assets/URP/RendererFeatures/RobotSensor/URPDepthEncoder.shader 使用
  LinearEyeDepth，随后乘 1000 并按低字节 R、高字节 G 编码 16UC1。
  这是前向深度，不是斜距；采样 UV 明确使用 (u,1-v)。
- RobotDepthFeature.cs 按 U16C1 设置毫米和 RG 编码，按 F32C1 设置米和 RGBA。
- RobotCameraSettings.cs 创建固定分辨率、无动态缩放、无 MSAA、Point 过滤的纹理。
- HSRPubXtionRGB.cs 内参硬编码 fx=fy=554,cx=320,cy=240，D 为零；图像
  frame_id 是相机父对象名称。读取 Texture 后直接发送原始字节。
- HSRPubTf.cs 对局部坐标使用位置 (-x,y,z)、四元数 (x,-y,-z,w) 转换。
  base_footprint TF 明确加入位置/角度噪声，因此 odom 中的轻微抖动不必然来自检测。

## 预制体证据

Competition/Common/Prefabs/HSR-B.prefab 继承 GUID
52e4f49510f5e83429982d5221396009，对应 SIGVerse/Models/Robot/HSR/Prefabs/HSR.prefab。
已读 HSR-B 覆盖列表，未见头部相机局部变换覆盖。

- head_rgbd_sensor_rgb_frame 和 head_rgbd_sensor_depth_frame 的父节点相同，
  局部位置均为零、旋转均为单位四元数。
- HeadRGBCamera 和 HeadDepthCamera 均在各自 frame 原点，局部旋转 (0,0,1,0)。
- 两相机尺寸 640x480、垂直 FOV 46.82 度、相同 culling mask 和视口。
- 由 FOV 推得 fy≈554.342，发布的 fy=554 有约 0.062% 差异；不能宣称精确标定。
- 深度相机 far=8 米、depthMinValue=0.4 米；RGB far=100 米。
  着色器对 depth<=0.4 或 depth>=far 输出零。

## 仍未完成的证据链

1. RGB Feature 使用材质 Blit，尚需沿有效 URP Renderer 配置追踪至 FlipY 图并确认实际启用。
2. 实际运行场景的 prefab 覆盖、运行时代码和另一台电脑版本需要核对。
3. 发布父 frame、相机子节点 180 度旋转与 TF 反射转换的光学轴关系需完整推导验证。
4. 透明物体可能不写入正常深度；不能把饮料罐验证直接推广至番茄酱透明外壳。

结论：副本中共视点配准与前向深度编码已有明确支持；尚不解除实时动作许可。
后续可以继续离线追踪配置。近距离抓取应另行设计深度失效处理，不应把零深度猜成距离。

## 后续核对：Renderer 引用与坐标推导

以下补充关闭上文待办 1，以及待办 3 的副本静态坐标推导部分：

- ProjectSettings/GraphicsSettings.asset 默认管线 GUID d1714af1a9830294abfeb9d061a65183
  对应 URP_VR.asset。已检查的全部八套 PipelineAssets，其 renderer 列表索引 1
  都指向 RobotSensor_Renderer（e6bf28fc583ecfb4683b7194328498fa）。
- HSR.prefab 两个头部相机的 UniversalAdditionalCameraData 均设 m_RendererIndex=1。
- RobotSensor_Renderer 内启用 RobotRgbFeature（be6736119edb89d4d8ad30eea434274b），
  shader 引用 e2a09dc062c150d40b0d5a4516cde14c，解析至 FlipYShaderGraph。
- BaseRobotSensorRenderFeature 以引用 shader 创建材质；RGB Feature 对 RGB 类型
  相机执行 Blit。Graph 连线为 ScreenPosition -> Split；X 直接到 Combine.R，
  Y 经 OneMinus 到 Combine.G；Combine.RG 接 SampleTexture2D.UV，纹理为 _BlitTexture。
  因此并非仅凭文件名判断，实际连线是 (u,1-v)。
- Unity Camera 中光学向量为 (x,-y,z)；子相机 Rz(pi) 后成为父框架 (-x,y,z)；
  发布端局部 X 反射后成为 (x,y,z)。据此父 frame 可承载右/下/前的光学向量，
  不需要在 ROS 端再额外旋转 180 度。此推导依赖已读到的相机局部变换没有覆盖。

新增 test_rgbd_optical_convention.py 验证上述基向量代数和上下翻转行号关系，
不将代数测试当成实际运行配置的认证。仍须核对另一台电脑的有效相机设置和
场景覆盖；geometry_verified 与 actionable 继续保持 false。
