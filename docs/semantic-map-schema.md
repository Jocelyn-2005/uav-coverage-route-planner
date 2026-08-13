# 云谷 Yungu2030 语义地图格式

本文记录当前解析器实际支持的输入格式。以下字段来自对源数据的检查，规划器不会把地图范围或建筑数量硬编码到算法中。

顶层字段为 `schema_version`、`world_name`、`coordinate_frame`、`units`、
`search_area`、`nodes` 和 `metadata`。

- 地图声明 schema `1.0`、坐标系 `ENU`、单位 `meters`；
- `search_area` 当前为四点矩形，实际边界始终从文件读取；
- 示例数据有 43 个节点：25 栋建筑、14 个区域和 4 个交通设施；
- 当前节点形状均为矩形，由 ENU 米制 `min_corner` 和 `max_corner` 描述；
- 节点属性包括 `category`、`type`、`label`、`passability`、`visibility`、
  `elevation_min_m` 和 `elevation_max_m`；
- `metadata` 记录真值排除状态和源资产。

当前解析器有意严格校验这一已观察格式。增加新图形类型或 schema 版本前，应先检查真实输入并为其增加模型与测试。
