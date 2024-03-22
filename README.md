使用NVIDIA DEEPSTREAM SDK 6.2 开发的扶梯监控系统。
在原DEEPSTREAM-APP的基础上，使用yolo8作为主检测模型，可以配置次级的关键点生成模型，使用rtmpose生成。
加入了光流生成插件和对光流场扰动分析算法，可在配置文件中启用。生成插件使用的是官方提供的nvof，依赖nvidia硬件，生成速度极快。
