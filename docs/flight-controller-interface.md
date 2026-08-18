# 飞控接口

单机任务读取 `flight_plan.json`，`flight_plan.yaml` 内容等价。协议使用局部 ENU 坐标和米制单位。

顶层包含 `schema_version`、`coordinate_frame`、`units`、`mission_status`、`map_id`、`video_detection`、`lanes`、`route_segments`、`waypoints` 和 `summary`。

`waypoints` 按 `sequence` 执行，包含位置、高度、航向、目标速度、原地转向和保持时间。最后一点速度为 0。`route_segments.kind` 区分 `coverage_lane`、`connector`、`obstacle_avoidance` 和 `return_home`。相机可全程提供视频，航段分类只用于任务解释和统计。

`video_detection.building_wall_occlusion=false` 表示本分支仅验证二维自由地面 footprint 覆盖，不声称完成建筑墙体遮挡可见性覆盖。`mission_status=infeasible_coverage` 表示仍有 patch 未满足阈值，不应直接下发执行。

飞控适配器应检查协议版本、ENU 原点、序号连续性、航段连通性，以及高度和速度是否符合载具限制。轨迹跟踪、平滑转弯、动力学约束、实时避障和降落流程由飞控或上层系统负责。

Web 双机任务输出 `mission_manifest.json` 以及 `drone_1/`、`drone_2/` 下的两份独立计划。清单不代表已经完成多机时空冲突消解，实际执行前仍需由上层系统检查。
