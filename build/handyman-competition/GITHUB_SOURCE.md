# 比赛包源码快照

此目录保存正式运行包的代码、配置、地图、几何数据、测试及使用说明。

Git 不保存运行日志、历史备份、Python/ROS 安装环境、编译产物和模型权重；仅克隆本仓库不能直接执行比赛启动脚本，需要配置相应运行依赖。完整可运行包仍保留在本机。模型标识与校验值见 runtime/active_detection_model.json，时钟桥源码在 workspace/training/session1/bridge_time_sync。迁移完整运行包后使用 resolve_package_paths.py 和 check_competition.sh 检查。
