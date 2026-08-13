# 通用运筹优化视角下的地图全覆盖摄影航点生成方法

`coverage-search-planner` 是一个面向无人机园区摄影搜索任务的纯 Python 规划器。它把任务建模为“连续空间几何覆盖 + 障碍约束路径规划 + 航线访问顺序优化”问题：在给定搜索区域、建筑语义、飞行高度、相机参数和安全距离的条件下，生成能够覆盖目标地面的连续航线和飞控途径点，并尽量降低总航程、非作业航程、转弯和不可达区域。

项目使用本地 ENU 米制坐标，不依赖 ROS、PX4、Gazebo、MAVROS 或 YOLO。规划结果通过版本化 JSON/YAML 协议交给下层飞控适配器。

## 快速验收

### 1. 克隆并安装

环境要求：Git、[`uv`](https://docs.astral.sh/uv/) 和 Python 3.12。依赖已经锁定在 `uv.lock`，不要安装到系统 Python。

```bash
git clone https://github.com/Jocelyn-2005/coverage-search-planner.git
cd coverage-search-planner
git switch feature/continuous-lane-planning
uv sync
uv run pytest -q
```

预期测试结果为全部通过。

### 2. 打开 Web 验收页面

```bash
uv run uvicorn coverage_planner.web:app --host 127.0.0.1 --port 8000
```

浏览器打开 <http://127.0.0.1:8000>。页面支持：

- 调整起降点 ENU 坐标和固定飞行高度；
- 配置扫描策略、扫描方向和建筑安全距离；
- 配置相机视场角、重叠率和固定拍摄频率；
- 配置覆盖、连接、避障速度和飞控途径点最大间距；
- 查看语义建筑、搜索网格、飞行路线和任务回放；
- 下载连续飞行计划、覆盖报告和 GeoJSON 文件。

验收建议：保持默认云谷参数，点击“开始规划”，确认地图出现闭合任务路线、均匀途径点和浅绿色摄影覆盖框，并检查结果区的覆盖率、总航程、航线数与未覆盖网格数。

### 3. 命令行生成验收结果

```bash
uv run coverage-planner plan \
  --semantic-map examples/yungu2030/semantic_map.json \
  --search-area examples/yungu2030/search_area.geojson \
  --config examples/yungu2030/planner_config.yaml \
  --output results/example_run
```

结果写入 `results/example_run/`。静态图可直接打开：

```text
results/example_run/visualization.png
```

## 输入

一次规划由三类输入组成。

| 输入 | 示例 | 含义 |
|---|---|---|
| 语义地图 | `semantic_map.json` | 地图边界、建筑矩形、建筑高度和类别 |
| 用户搜索区域 | `search_area.geojson` | 需要摄影覆盖的 Polygon/MultiPolygon |
| 规划配置 | `planner_config.yaml` | 起点、高度、相机、重叠率、安全距离、速度和采样参数 |

核心坐标约定：

- 坐标系为局部 ENU；`x` 向东，`y` 向北，`z` 向上；
- 距离、高度和坐标单位均为米；
- 像素坐标只用于经过标定的 Web 显示，不参与核心规划；
- 当前版本只支持固定高度、垂直向下相机，即 `pitch_deg = -90`。

有效搜索地面定义为：

```text
用户搜索区域 ∩ 地图边界 − 建筑占地 − 显式排除区域
```

建筑是否构成飞行障碍由飞行高度、建筑高度、垂直净空和 `allow_overflight_above_buildings` 共同决定。规划器不会自动升高无人机。

完整地图格式见 [语义地图格式](docs/semantic-map-schema.md)。

## 输出

| 文件 | 使用方 | 作用 |
|---|---|---|
| `flight_plan.json` | 下层飞控适配器，首选 | 连续摄影飞行协议 v2 |
| `flight_plan.yaml` | 人工审阅/飞控适配器 | 与 JSON 等价的 YAML |
| `waypoints.json` | 旧系统兼容 | 旧版规划关键点和相机 footprint |
| `waypoints.csv` | 表格检查 | 旧版航点表 |
| `coverage_report.json` | 验收与实验统计 | 覆盖率、航程、非作业距离、不可达网格 |
| `patches.geojson` | GIS/覆盖分析 | 每个地面网格的覆盖状态 |
| `route.geojson` | GIS/地图显示 | 任务路线折线 |
| `visualization.png` | 人工验收 | 静态规划结果图 |

### 给下层飞控的文件

下层飞控应优先读取 `flight_plan.json`，而不是旧版 `waypoints.json`。协议包含：

- `waypoints`：均匀细分后的 ENU 飞控途径点，带高度、航向和目标速度；
- `route_segments`：相邻途径点之间的直线航段；
- `kind`：`coverage_lane`、`connector`、`obstacle_avoidance` 或 `return_home`；
- `capture_enabled`：该航段是否按固定频率连续拍摄；
- `lanes`：覆盖航线与其航段编号；
- `control_point_spacing_m`：途径点最大间距；
- `summary`：覆盖与任务效率指标。

飞控适配器的基本执行规则为：依序跟踪 `waypoints`，在当前 `route_segment.capture_enabled = true` 时按 `capture.frequency_hz` 触发相机，在航段切换时采用该段的 `heading_deg` 和 `speed_mps`。控制点不是单次拍照触发点。

详细字段和执行状态机见 [下层飞控接口](docs/flight-controller-interface.md)。

## 方法概览

相机在高度 `h` 下形成地面 footprint；重叠率决定扫描线间距和最大摄影间距。规划器先对有效搜索多边形生成平行扫描线或轮廓候选，然后进行补漏、建筑安全缓冲裁剪、航线方向/顺序选择、可见图避障连接、连续摄影覆盖复核，最后均匀细分为飞控途径点。

运筹优化视角下，主要决策包括扫描方向、航线集合、每条航线的执行方向、访问顺序、连接路径、拍摄开关和速度。目标是在满足覆盖、安全、闭合任务和飞控跟踪约束的前提下，综合最小化总航程、非作业航程、转弯代价和未覆盖惩罚。

- [问题定义与数学模型](docs/optimization-model.md)
- [几何与优化算法设计](docs/algorithm-design.md)
- [下层飞控接口](docs/flight-controller-interface.md)
- [后续设计边界](docs/future-design.md)

## 当前能力边界

当前版本是确定性的分层启发式求解器，不宣称获得全局最优解。它已经实现固定高度正射覆盖、两种扫描模式、建筑高度相关避障、可见图最短路、连续摄影评估、均匀飞控途径点和多种结果导出。全局航线排序、最小转弯半径、曲率连续轨迹、地形跟随、斜视相机、三维遮挡和 GSD 高度约束仍是后续研究方向。
