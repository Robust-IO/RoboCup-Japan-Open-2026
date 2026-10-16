将 Label Studio 导出的 YOLO 数据直接放到这个目录下。

期望结构：

```text
training/data/raw/
  images/
  labels/
  classes.txt
  notes.json
```

注意事项：

- `classes.txt` 必须与 `training/classes.txt` 完全一致
- `labels/` 中每个 `.txt` 文件必须与 `images/` 中同名图片对应
- 没有目标的负样本可以保留空 `.txt`
- `notes.json` 作为 Label Studio 附带元数据保留即可
