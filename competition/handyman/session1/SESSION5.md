# Session5 独立全自动测试

官方 EnvironmentInfo05.json：LayoutD。
任务：Go to the kitchen, grasp the rabbit_doll and put it on the square_low_table.
用户确认源家具：LayoutD/SIGVerseHouse/kitchen/wooden_cupboard/body，非 blue_cupboard/body。

流程：进入厨房并深入区域0.5米后上报 → 靠近木柜（底盘中心距家具边缘1米）立即识别 → RGBD定位rabbit_doll → 通用抓取及抬升夹持验证 → 撤离收臂 → 导航到客厅square_low_table → 测量支撑面并寻找空位 → 放置并验证高度及连续帧稳定 → Task_finished → 等待Moderator判定。

源柜体不在原DestinationCandidates中。session5_source_geometry.py只解析具名静态木柜的变换与顶部BoxCollider，不读取目标物品或生成点坐标。prepare_session5_config.py生成独立的layout_d_session5.yaml和session5-semantic-map.json，原始导出不变。目标位置依然来自相机。

已修正共享navigate_destination使用destination_room，避免跨房间搬运仍导航到源房间。

在Unity切换Session5并回到该轮初始位置后运行：

```bash
cd /__HANDYMAN_PACKAGE__/workspace/training/session1
bash start_session5_round.sh --execute --from-session5-initial
```

日志：D:\handyman-training-runs\session5-auto-时间戳，最新目录指针session5-latest.txt。
不要在其他自动任务运行时启动。已接入Session1～5串联，入口 start_sessions12345.sh --execute --from-session1-initial。第四轮后等待LayoutD的新握手再启动第五轮。第五轮搬运放置动作在断点恢复测试完成；新动作完成即上报规则及五轮连续切换仍待实测。

离线核查：静态木柜顶面约0.70685米；26个可达且无遮挡的源候选；3个可达目的地候选；目的桌官方坐标与导出一致。这些是静态检查，不等于抓取放置实测通过。
