# UAV Coverage Generation and Route Optimization Planner

面向无人机园区连续视频检测任务的 **Coverage Generation + Route Optimization** 规划器。给定搜索责任区、2.5D 建筑地图、相机与飞行参数以及起降点，系统生成全覆盖航道，优化航道访问顺序与方向，展开绕障安全路线，并通过连续视野复核与自动补漏生成最终可执行飞行计划。

当前目标为地面静止人员和车辆，采用固定高度、垂直向下相机和 2.5D 建筑模型。目标检测算法本身不在本仓库范围内；理想检测模型规定，目标完整进入有效相机视野一次即认为可检测。

项目使用本地 ENU 米制坐标，不依赖 ROS、PX4、Gazebo、MAVROS 或 YOLO。最终结果通过版本化 JSON/YAML 协议交给下层飞控适配器。

## Pipeline

```text
Inputs
  ↓
① Primary Coverage Generation
   Global Scanline / BCD
  ↓
CoveragePlan
  ↓
② Lane Ordering & Orientation
   obstacle-aware transition cost
  ↓
Ordered & Oriented Coverage Lanes
  ↓
③ Obstacle-aware Route Expansion
   connector / avoidance / return-home
  ↓
Candidate Safe Flight Route
  ↓
④ Continuous Visibility Evaluation
   patch-wise coverage check
  ↓
Coverage Requirement Satisfied?
  ├─ Yes → mission_status = ready
  └─ No  → ⑤ Coverage Completion
                 ↓
            update route
                 ↓
             ③ → ④
````

Coverage Generation 负责回答“**需要扫哪些航道**”；Route Optimization 负责回答“**这些航道按什么顺序和方向飞**”；Route Expansion 将结果展开为实际安全路径；Visibility Evaluation 检查完整飞行过程中真正获得的连续视频覆盖；若仍有漏扫区域，则 Coverage Completion 自动补全并重新复核。

## Quick Start

### 1. Clone & Install

环境要求：Git、Python 3.12 和 [`uv`](https://docs.astral.sh/uv/)。

```bash
git clone https://github.com/Jocelyn-2005/uav-coverage-route-planner.git
cd uav-coverage-route-planner
git switch feature/continuous-lane-planning

uv sync
uv run pytest -q
```

依赖已锁定在 `uv.lock`，建议使用 `uv` 创建的虚拟环境，不要安装到系统 Python。

### 2. Start Web UI

```bash
uv run uvicorn coverage_planner.web:app --host 127.0.0.1 --port 8000
```

浏览器打开：

```text
http://127.0.0.1:8000
```

Web 页面可以：

* 绘制一架或两架无人机的搜索责任区并设置起降点；
* 在 Global Scanline 与 BCD 之间选择 Coverage Generation 方法；
* 配置扫描方向、相机 FOV、目标包络、Overlap、安全距离、飞行高度和速度；
* 在 Local Insertion 与 Full Greedy 之间选择 Coverage Completion 策略；
* 查看 Coverage Lanes、Connector、Obstacle Avoidance、Return Home 和 Coverage Patches；
* 回放完整任务并下载 Flight Plan、Coverage Report 和 GeoJSON。

推荐验收方式：绘制两个互不重叠责任区，点击生成并优化航线，确认两架无人机分别得到从 Home 出发并返回 Home 的闭合安全路线，同时最终 Required Patches 均满足覆盖阈值。

> 双机当前仅进行独立责任区规划，不执行两机之间的时空碰撞规避。

### 3. CLI Planning

```bash
uv run coverage-planner plan \
  --semantic-map examples/yungu2030/semantic_map.json \
  --search-area examples/yungu2030/search_area.geojson \
  --config examples/yungu2030/planner_config.yaml \
  --output results/example_run
```

主要结果写入：

```text
results/example_run/
├── flight_plan.json
├── flight_plan.yaml
├── coverage_report.json
├── patches.geojson
├── route.geojson
└── visualization.png
```

其中 `flight_plan.json` 是推荐提供给下层飞控适配器的主要协议文件。

## Inputs

一次规划主要包括以下输入：

| 输入               | 内容                                               |
| ---------------- | ------------------------------------------------ |
| Search Area      | 每架无人机负责搜索的 Polygon / MultiPolygon                |
| Buildings (2.5D) | 建筑 Footprint、Height 和类别                          |
| Camera & Video   | Resolution、HFOV/VFOV、Overlap、目标宽/长/高、边缘余量、视频分析频率 |
| Flight Params    | 飞行高度、安全距离、速度等                                    |
| Home / Start     | 起飞与返航位置                                          |

坐标采用局部 ENU：

```text
x → East
y → North
z → Up
```

距离、高度和坐标单位均为米。当前相机固定垂直向下，即：

```text
pitch_deg = -90°
```

有效搜索地面定义为：

```text
Effective Search Area
=
Search Area
∩ Map Boundary
− Building Footprints
− Explicit Excluded Regions
```

建筑是否构成飞行障碍，由建筑高度、飞行高度、安全净空以及是否允许从建筑上方飞越共同决定。

## Outputs

| 文件                     | 作用                         |
| ---------------------- | -------------------------- |
| `flight_plan.json`     | 下层飞控适配器使用的连续任务协议           |
| `flight_plan.yaml`     | JSON 的 YAML 等价版本，便于人工查看    |
| `coverage_report.json` | 覆盖率、航程、未覆盖区域等任务指标          |
| `patches.geojson`      | Patch-wise Coverage Result |
| `route.geojson`        | 完整任务路线                     |
| `visualization.png`    | 静态规划结果图                    |

双机 Web 任务额外生成：

```text
mission_manifest.json
drone_1/
drone_2/
```

最终主要任务状态为：

```text
mission_status = ready
mission_status = infeasible_coverage
```

`ready` 表示所有需要强制验收的 Required Patches 均达到覆盖要求；`infeasible_coverage` 表示经过补全后仍存在无法满足要求的 Required Patch，该路线不得直接下发执行。

---

# Method

## 1. Primary Coverage Generation

Coverage Generation 根据 Effective Search Area、Effective Camera Footprint 和 Overlap 生成理论覆盖骨架。

当前提供两种并列方法：

### Global Scanline

在整个有效搜索区域建立统一平行扫描线，再通过搜索边界和障碍进行几何裁剪：

```text
Effective Search Area
→ Global Scanlines
→ Geometry Clipping
→ Coverage Lanes
```

### BCD — Boustrophedon Cellular Decomposition

首先根据扫描过程中发生的拓扑 Split / Merge 将区域分解为 Boustrophedon Cells，再在各 Cell 内生成 Lawnmower Coverage Lanes：

```text
Topology Sweep
→ BCD Cells
→ Cell-wise Lawnmower Lanes
```

两种方法最终输出统一的：

```text
CoveragePlan
├─ Coverage Lanes
├─ Reference Waypoints
├─ Scan Direction
└─ BCD Cell Index    # BCD only
```

这一阶段只生成 Coverage Lanes，不负责 Lane 之间的连接、绕障或返航。

## 2. Lane Ordering & Orientation

给定 Section 1 生成的 Coverage Lanes 后，Route Optimization 联合决定：

```text
Lane Ordering
+
Lane Orientation
```

规划器首先基于建筑飞行障碍建立 Visibility Graph，计算：

```text
Home ↔ Lane Endpoints
Lane Endpoints ↔ Lane Endpoints
```

之间的安全最短路径距离，并形成 **Obstacle-aware Transition Cost**。

因此优化使用的不是端点欧氏距离，而是实际绕障后的安全连接距离。

当前目标可概括为：

```text
minimize
Obstacle-aware Transition Distance
+
Return-to-Home Distance
```

Coverage Lane 集合本身在该阶段保持不变。

求解器自动选择：

```text
≤ 12 lanes → Exact
> 12 lanes → Greedy + 2-opt + Or-opt
```

其中 Exact 仅表示对固定 Coverage Lane 集合的访问顺序与方向进行精确求解，并不代表整个规划流程全局最优。

## 3. Obstacle-aware Route Expansion

Section 2 只决定 Lane 的顺序和方向，并不直接生成详细绕障轨迹。

顺序确定后，Section 3 使用 Visibility Graph Shortest Path 将选中的转场展开为完整路线：

```text
Coverage Lane
Connector
Obstacle Avoidance
Return Home
```

得到：

```text
Candidate Safe Flight Route
```

其中 Connector 表示可以直接安全连接的航段，Obstacle Avoidance 表示因建筑阻挡而产生的绕障航段，Return Home 表示任务结束后的安全返航路径。

## 4. Continuous Visibility Evaluation

理论 Coverage Lanes 并不等于最终任务一定满足覆盖要求。因此完整安全路线生成后，规划器会沿所有航段重新进行连续视频视野复核。

根据：

```text
Flight Speed
+
Video Analysis Rate
```

对路线进行离散采样，并结合 Camera Footprint 与 Building Occlusion 计算实际可见地面。

以下航段产生的有效视野都可以计入 Coverage：

```text
Coverage Lane
Connector
Obstacle Avoidance
Return Home
```

Effective Search Area 在规划开始时划分为 Coverage Patches。每个 Required Patch 默认要求：

```text
Coverage Ratio ≥ 99%
```

只有全部 Required Patches 均满足要求时：

```text
mission_status = ready
```

裁剪后面积小于标准 Patch 面积 5% 的边界碎片仍参与 Coverage Generation、Completion 和统计，但不影响最终成功状态。

## 5. Coverage Completion

若 Continuous Visibility Evaluation 发现残余漏扫区域，则进入 Coverage Completion。

系统针对未满足要求的 Patches 生成 Completion Points，并综合考虑：

```text
Visibility Gain
Insertion Cost
```

即优先选择能够覆盖更多漏扫区域、同时增加较少额外航程的补全点。

当前支持两种策略。

### Local Insertion

默认策略：

```yaml
completion_strategy: local_insertion
```

保持原 Coverage Lane 的顺序和方向不变，只在 Lane / Job 边界插入 Completion Point，然后重新执行 Route Expansion。

适合对已经优化好的主路线进行小规模增量修复。

### Full Greedy

```yaml
completion_strategy: full_greedy
```

将 Coverage Lanes 与 Completion Points 一起作为 Route Jobs，重新计算 Job Ordering 和 Lane Orientation。

相比 Local Insertion，其路线调整范围更大，可作为高成本对照策略。

每轮补全后重新执行：

```text
Coverage Completion
        ↓
Obstacle-aware Route Expansion
        ↓
Continuous Visibility Evaluation
```

最多执行 10 轮。若最终仍存在：

```text
Required Patch Coverage Ratio < 99%
```

则返回：

```text
mission_status = infeasible_coverage
```

---

# Flight Controller Interface

下层飞控优先读取：

```text
flight_plan.json
```

协议主要包含：

```text
waypoints
route_segments
lanes
control_point_spacing_m
summary
mission_status
```

`route_segments[].kind` 包括：

```text
coverage_lane
connector
obstacle_avoidance
return_home
```

`detection_enabled` 表示对应航段产生的视频检测结果是否计入当前搜索任务，它不是相机快门触发指令。

完整路线按照 `control_point_spacing_m` 均匀细分为飞控 Waypoints。对于尖锐折返位置，可以通过 `turn_in_place=true` 和 `hold_time_s` 要求多旋翼悬停并调整航向后再进入下一航段。

详细协议见 [`docs/flight-controller-interface.md`](docs/flight-controller-interface.md)。

# Current Scope

当前版本属于确定性的分层 Coverage / Route Planner，已经实现：

* Global Scanline 与 BCD Coverage Generation；
* Camera-aware Lane Spacing；
* Lane Ordering 与 Orientation；
* Visibility Graph 障碍感知转场成本；
* Exact / Greedy / 2-opt / Or-opt；
* Connector、Obstacle Avoidance 与 Return Home；
* Building Occlusion；
* Continuous Video Visibility Evaluation；
* Patch-wise Coverage Validation；
* Local Insertion / Full Greedy Coverage Completion；
* 单机及双机独立责任区规划；
* Flight-control Waypoint Export。

当前暂不处理：

* Moving Targets；
* Multi-UAV Spatiotemporal Collision Avoidance；
* Terrain Following；
* Oblique Camera；
* Dynamic Obstacles；
* Arbitrary 3D Building Mesh；
* Full Vehicle Dynamics；
* Dynamically Feasible Trajectory Smoothing；
* 端到端全局最优 Coverage + Routing。

整体设计原则可以概括为：

```text
Coverage Geometry
      ↓
Route Optimization
      ↓
Safe Route Expansion
      ↓
Mission-level Visibility Validation
      ↓
Coverage Completion if Necessary
```

其中 **CoveragePlan 只是理论覆盖骨架，Candidate Safe Flight Route 只是安全候选路线；只有经过完整 Continuous Visibility Evaluation 并满足 Patch-wise Coverage Requirement 的路线，才被标记为最终可执行任务。**

# Documentation

* [`docs/optimization-model.md`](docs/optimization-model.md) — 问题定义与数学模型
* [`docs/algorithm-design.md`](docs/algorithm-design.md) — 几何与优化算法
* [`docs/benchmark-design.md`](docs/benchmark-design.md) — Coverage Generation × Route Optimization Benchmark
* [`docs/flight-controller-interface.md`](docs/flight-controller-interface.md) — 下层飞控接口
* [`docs/semantic-map-schema.md`](docs/semantic-map-schema.md) — 语义地图格式
* [`docs/future-design.md`](docs/future-design.md) — 后续设计边界

```
