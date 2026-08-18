# 低算力无人机全覆盖航线规划器

本分支面向 Jetson Nano 等资源受限设备，在已知地图和起飞前离线规划条件下，为两架无人机生成固定高度覆盖航线与飞控航点，并逐 patch 验收覆盖结果。

## 当前实现

- 有效搜索地面：`用户区域 ∩ 地图范围 − 建筑占地 − 显式排除区`；
- Coverage Generation：仅使用 Global Scanline；
- 扫描方向：`0°`、`90°`，或 Auto 在两者中选择；
- Route Optimization：Greedy 决定 lane 顺序和正反方向；
- 连接：使用二维建筑安全缓冲和障碍最短路；
- 输出：均匀飞控航点、分类航段、覆盖报告和 GeoJSON；
- 验收：按垂直向下相机 footprint 检查二维自由地面几何覆盖。

本分支不实现 BCD、Exact、2-opt、Or-opt、补全观察点、建筑墙体遮挡投影、动态补漏、轨迹平滑或双机时空冲突消解。Web 接收两个互不重叠的责任区，内部执行两次独立轻量规划。

## 安装与检查

需要 Python 3.12 和 `uv`：

```bash
uv sync
uv run pytest -q
uv run ruff check src tests
uv run mypy src
```

## Web

```bash
./scripts/start_web.sh
```

默认访问 <http://127.0.0.1:8765>。页面可绘制两个责任区、设置两个起点，显示两架无人机的分类航段、同步飞行动画和随飞行逐步出现的覆盖范围。

## CLI

```bash
uv run coverage-planner plan \
  --semantic-map examples/yungu2030/semantic_map.json \
  --search-area examples/yungu2030/search_area.geojson \
  --config examples/yungu2030/planner_config.yaml \
  --output results/example_run
```

单机主要输出为 `flight_plan.json`、`flight_plan.yaml`、`coverage_report.json`、`patches.geojson` 和 `route.geojson`。双机 Web 任务额外输出 `mission_manifest.json`，并在 `drone_1/`、`drone_2/` 下保存两份独立任务。若存在未满足阈值的 patch，对应结果会明确标记为 `infeasible_coverage`，不会伪装为可执行任务。

## 能力边界

当前保证对象是二维可搜索自由地面的几何覆盖，不是建筑遮挡后的严格可见性覆盖。规划器只检查航点、速度、航段连通性和二维安全净空；具体轨迹跟踪、平滑转弯、动力学限制和实时避障由飞控或上层系统负责。

输入格式见 [语义地图格式](docs/semantic-map-schema.md)，输出协议见 [飞控接口](docs/flight-controller-interface.md)，算法见 [算法设计](docs/algorithm-design.md)。
