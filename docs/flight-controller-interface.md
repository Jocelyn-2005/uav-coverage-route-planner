# 下层飞控接口说明

## 1. 交付文件

下层飞控适配器应读取规划输出目录中的 `flight_plan.json`。`flight_plan.yaml` 内容等价，适合人工审阅。

协议当前版本为 `schema_version = "2.0"`，坐标系为 ENU，单位为米。

## 2. 顶层结构

```json
{
  "schema_version": "2.0",
  "coordinate_frame": "ENU",
  "units": "meters",
  "map_id": "yungu2030_local_origin",
  "capture": {},
  "lanes": [],
  "route_segments": [],
  "waypoints": [],
  "summary": {}
}
```

## 3. 摄影配置

```json
{
  "mode": "continuous",
  "frequency_hz": 2.0,
  "control_point_spacing_m": 10.0,
  "forward_overlap": 0.3,
  "lane_overlap": 0.3
}
```

- `mode`：当前固定为连续摄影；
- `frequency_hz`：启用摄影的航段内固定触发频率；
- `control_point_spacing_m`：规划输出允许的最大相邻途径点间距；
- `forward_overlap`、`lane_overlap`：规划使用的航向/旁向重叠率。

## 4. 飞控途径点

```json
{
  "id": "fp_0001",
  "sequence": 1,
  "x": 153.4,
  "y": 67.2,
  "z": 30.0,
  "heading_deg": 90.0,
  "speed_mps": 4.0
}
```

- `sequence` 从 1 开始连续递增；
- `x/y/z` 为局部 ENU 目标位置；
- `heading_deg` 为该点之后航段的目标航向，0° 指北、90° 指东；
- `speed_mps` 为该点之后航段的目标速度；
- 最后一个点速度为 0。

## 5. 航段

```json
{
  "id": "segment_0001",
  "sequence": 1,
  "kind": "connector",
  "start_waypoint_id": "fp_0001",
  "end_waypoint_id": "fp_0002",
  "heading_deg": 90.0,
  "speed_mps": 4.0,
  "length_m": 8.7,
  "capture_enabled": false
}
```

`kind` 枚举值：

| 值 | 含义 |
|---|---|
| `coverage_lane` | 主覆盖作业航段 |
| `connector` | 覆盖航线之间的连接段 |
| `obstacle_avoidance` | 建筑安全缓冲绕行段 |
| `return_home` | 返回起降点航段 |

`capture_enabled` 是相机控制的唯一权威字段。它通常在覆盖航段为 `true`，但当连接或返航能够补充未覆盖地面时，也可能为 `true`。

## 6. 推荐执行状态机

```text
加载并校验协议
      ↓
飞往 waypoints[0]
      ↓
按 sequence 读取 route_segment
      ↓
设置目标速度和航向
      ↓
capture_enabled ? 开启固定频率触发 : 停止触发
      ↓
跟踪 end_waypoint_id 对应的 ENU 点
      ↓
到达容差内 → 下一航段
      ↓
最后一点悬停/降落并关闭相机
```

适配器必须在任务开始前校验：

1. schema 版本、坐标系和单位受支持；
2. waypoint `sequence` 连续且 ID 唯一；
3. segment `sequence` 连续；
4. 每个 segment 的首尾 ID 存在；
5. 相邻 segment 连通；
6. 高度、速度和拍摄频率在载具能力范围内；
7. 本地 ENU 原点与规划地图一致。

## 7. 飞控不应自行推断的内容

- 不应根据航点是否为转折点决定是否拍照；
- 不应根据 `kind` 代替 `capture_enabled`；
- 不应自动修改 ENU 原点、轴方向或单位；
- 不应在未重新进行安全校验时跳过中间途径点；
- 不应把规划 footprint 当作实时定位真值。

## 8. 容错建议

- 对每个途径点设置水平、垂直到达容差和超时；
- 超出走廊或定位质量不足时暂停摄影并进入安全策略；
- 相机触发应记录时间戳、ENU/全球坐标、航向和任务航段 ID；
- 中断恢复时从安全的 segment 边界恢复，而不是仅凭最近 waypoint ID；
- 实际载具若有最小转弯半径，应在适配层或上游轨迹平滑模块中明确处理，不能假设当前折线天然满足动力学约束。

## 9. 其他输出的用途

- `coverage_report.json`：任务验收和实验指标，不用于实时控制；
- `route.geojson`：GIS 显示；
- `patches.geojson`：覆盖缺口分析；
- `visualization.png`：人工快速检查；
