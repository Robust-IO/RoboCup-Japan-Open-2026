# V4 六类：番茄酱与饮料罐区分

已核实旧现场所谓“垃圾桶图案”实际上是 filled_ketchup#1。此前针对该区域的垃圾桶误检结论作废；历史现场图不能作为独立最终测试集，需重新采集。

映射（pilot ID / training/classes.txt ID）：apple 0/0，canned_juice 1/3，rabbit_doll 2/15，pink_cup 3/14，white_cup 4/26，filled_ketchup 5/9。27 类表编号不是所有旧节点通用的编号，不修改旧 ROS 节点。

## Unity 小批量

1. 退出 Play，备份 Assets/HandymanDatasetGenerator。用 HandymanDatasetGenerator-v4.zip 中的 Assets 内容覆盖同名工具，保留 .meta。
2. 等待编译无错误。打开生成器确认 V4，点击 Find Missing Sources；原五项保留，新增 filled_ketchup 应选实际 filled_ketchup#1 模型根节点，不是 empty_ketchup。
3. 保留现有墙、地板、相机和背景家具。输出新目录 D:/handyman-dataset/pilot08，Frame count=90，seed=20260918。
4. 导出后传回 ROS 电脑同一路径。保留所有旧批次。

生成器轮换单物品、juice/ketchup 同框和混合场景；是否实际可见仍需检查 mask，不能只根据采样计划判定覆盖。新实例 ID 6 使用青色 (0,255,255)，背景仍为 0。

## Ubuntu QA

使用新的 training/scripts/prepare_unity_six.py --inspect /mnt/d/handyman-dataset/pilot08 --output 一个全新目录。它严格要求六类 batch registry。旧 prepare_unity_pilot.py 仍只接受五类，已有 YAML 和权重不改动。

检查六类计数、番茄酱与饮料罐单独/同框可见情况、边缘位置和标签对齐。旧五类原始批次不直接加入六类转换，避免出现未标注番茄酱的旧画面。QA 通过后才规划新训练。

只读推理支持严格的原五类或新六类模型，并输出 global_class_id（27 类训练表编号）。尚无六类权重，代码更新不会把五类模型自动变成六类。后续六类训练需要新建运行目录并验证分类头和 names。

Unity 实际编译/渲染需用户环境验证。现有 Self Check 的 v3 日志只覆盖命名/投影数学，不代表六类渲染已经通过。
