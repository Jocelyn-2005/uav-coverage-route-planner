const svg = document.querySelector('#canvas');
const planButton = document.querySelector('#plan');
const summaryBox = document.querySelector('#summary');
const exportBox = document.querySelector('#exports');
const replayButton = document.querySelector('#replay');
const ns = 'http://www.w3.org/2000/svg';
let mapData;
let planData;
let animationFrame;
let animationState = null;

const strategyNames = {
  auto: '自动比较',
  contour_outward: '由内向外轮廓搜索',
  lawn_mower: '往复式覆盖搜索',
};
const exportNames = {
  'flight_plan.json': '连续飞行计划（JSON）',
  'flight_plan.yaml': '连续飞行计划（YAML）',
  'waypoints.json': '兼容航点（JSON）',
  'waypoints.csv': '兼容航点（CSV）',
  'patches.geojson': '覆盖网格（GeoJSON）',
  'route.geojson': '飞行路线（GeoJSON）',
  'coverage_report.json': '覆盖报告（JSON）',
  'visualization.png': '规划结果图（PNG）',
};

const input = id => document.querySelector(`#${id}`);
const layer = name => document.querySelector(`[data-layer=${name}]`).checked;

function element(tag, attributes, className) {
  const node = document.createElementNS(ns, tag);
  Object.entries(attributes).forEach(([key, value]) => node.setAttribute(key, value));
  if (className) node.setAttribute('class', className);
  return node;
}

function polygonPoints(coordinates) {
  return coordinates.map(point => `${point[0]},${-point[1]}`).join(' ');
}

function rings(geometry) {
  if (geometry.type === 'Polygon') return [geometry.coordinates[0]];
  if (geometry.type === 'MultiPolygon') return geometry.coordinates.map(polygon => polygon[0]);
  return [];
}

function draw() {
  svg.innerHTML = '';
  if (!mapData) return;
  if (layer('background')) {
    const [minX, minY, maxX, maxY] = mapData.background.bounds;
    svg.append(element('image', {
      href: `${mapData.background.url}?v=3`, x: minX, y: -maxY,
      width: maxX - minX, height: maxY - minY, preserveAspectRatio: 'none',
    }));
  }
  svg.append(element('polygon', {
    points: polygonPoints(mapData.search_area.coordinates[0]),
    style: 'fill:none;stroke:#c92a2a;stroke-width:.8px',
  }, 'boundary'));
  if (layer('buildings')) mapData.buildings.forEach(building => {
    const [minX, minY, maxX, maxY] = building.bounds;
    const rectangle = element('rect', {
      x: minX, y: -maxY, width: maxX - minX, height: maxY - minY,
    }, 'building');
    const title = element('title', {});
    title.textContent = `${building.id} · 高度 ${building.height_m.toFixed(1)} 米`;
    rectangle.append(title);
    svg.append(rectangle);
  });
  if (planData && layer('patches')) planData.patches.forEach(patch =>
    rings(patch.geometry).forEach(ring => svg.append(element('polygon', {
      points: polygonPoints(ring),
    }, `patch ${patch.covered ? '' : 'uncovered'}`))));
  const flightRoute = planData ? (planData.flight_waypoints.length ? planData.flight_waypoints : planData.waypoints) : [];
  if (planData && layer('route')) svg.append(element('polyline', {
    points: flightRoute.map(waypoint => `${waypoint.x},${-waypoint.y}`).join(' '),
  }, 'route'));
  if (planData && layer('waypoints')) flightRoute.forEach(waypoint => {
    const circle = element('circle', {
      cx: waypoint.x, cy: -waypoint.y, r: .9,
    }, 'waypoint');
    const title = element('title', {});
    title.textContent = `${waypoint.id} · ENU ${waypoint.x.toFixed(2)}, ${waypoint.y.toFixed(2)}, ${waypoint.z.toFixed(2)} · 航向 ${waypoint.heading_deg?.toFixed(1) ?? waypoint.yaw_deg.toFixed(1)}° · 速度 ${waypoint.speed_mps?.toFixed(1) ?? '-'} 米/秒`;
    circle.append(title);
    svg.append(circle);
  });
  const home = [+input('home-x').value, +input('home-y').value];
  if (Number.isFinite(home[0]) && Number.isFinite(home[1])) {
    svg.append(element('circle', { cx: home[0], cy: -home[1], r: 2 }, 'home'));
    const title = element('title', {}); title.textContent = `起降点 · ENU ${home[0]}, ${home[1]}`;
    svg.lastChild.append(title);
  }
  drawAnimationState();
}

function drawAnimationState() {
  if (!animationState || !planData) return;
  animationState.footprints.forEach(ring => svg.append(element('polygon', {
    points: polygonPoints(ring),
  }, 'captured-footprint')));
  if (animationState.currentFootprint) svg.append(element('polygon', {
    points: polygonPoints(animationState.currentFootprint),
  }, 'captured-footprint'));
  if (animationState.travelled.length > 1) svg.append(element('polyline', {
    points: animationState.travelled.map(point => `${point[0]},${-point[1]}`).join(' '),
  }, 'travelled'));
  if (animationState.position) svg.append(element('circle', {
    cx: animationState.position[0], cy: -animationState.position[1], r: 1.7,
  }, 'drone'));
}

function cameraFootprint(position, headingDeg) {
  const altitude = +input('alt').value;
  const width = 2 * altitude * Math.tan(+input('hfov').value * Math.PI / 360);
  const length = 2 * altitude * Math.tan(+input('vfov').value * Math.PI / 360);
  const yaw = headingDeg * Math.PI / 180;
  const forward = [Math.sin(yaw), Math.cos(yaw)];
  const right = [Math.cos(yaw), -Math.sin(yaw)];
  return [[1,1],[1,-1],[-1,-1],[-1,1]].map(([f,r]) => [
    position[0] + f * length / 2 * forward[0] + r * width / 2 * right[0],
    position[1] + f * length / 2 * forward[1] + r * width / 2 * right[1],
  ]);
}

function replayMission() {
  if (!planData || planData.flight_waypoints.length < 2) return;
  cancelAnimationFrame(animationFrame);
  const route = planData.flight_waypoints;
  animationState = { segment: 0, progress: 0, footprints: [], travelled: [[route[0].x, route[0].y]],
    position: [route[0].x, route[0].y], currentFootprint: null, lastCaptureDistance: 0, lastTime: null };
  replayButton.disabled = true;
  function step(time) {
    if (animationState.lastTime === null) animationState.lastTime = time;
    const elapsed = Math.min(50, time - animationState.lastTime);
    animationState.lastTime = time;
    const from = route[animationState.segment];
    const to = route[animationState.segment + 1];
    const distance = Math.hypot(to.x - from.x, to.y - from.y);
    const metresPerSecond = 10 + Number(input('speed').value) * 18;
    animationState.progress += distance === 0 ? 1 : elapsed / 1000 * metresPerSecond / distance;
    const ratio = Math.min(1, animationState.progress);
    animationState.position = [from.x + (to.x - from.x) * ratio, from.y + (to.y - from.y) * ratio];
    const captureEnabled = planData.route_segments[animationState.segment]?.capture_enabled ?? true;
    animationState.currentFootprint = captureEnabled
      ? cameraFootprint(animationState.position, from.heading_deg) : null;
    const travelledOnSegment = distance * ratio;
    const captureSpacing = Math.max(.5, from.speed_mps / +input('frequency').value);
    if (captureEnabled && travelledOnSegment - animationState.lastCaptureDistance >= captureSpacing) {
      animationState.footprints.push(animationState.currentFootprint);
      animationState.lastCaptureDistance = travelledOnSegment;
    }
    if (ratio === 1) {
      animationState.travelled.push([to.x, to.y]);
      animationState.segment += 1;
      animationState.progress = 0;
      animationState.lastCaptureDistance = 0;
      if (animationState.segment >= route.length - 1) {
        draw(); replayButton.disabled = false; return;
      }
    }
    draw();
    animationFrame = requestAnimationFrame(step);
  }
  draw();
  animationFrame = requestAnimationFrame(step);
}

function showError(error) {
  const box = input('error');
  box.textContent = error instanceof Error ? error.message : String(error);
  box.style.display = 'block';
}

document.querySelectorAll('[data-layer]').forEach(control => { control.onchange = draw; });
['home-x', 'home-y'].forEach(id => { input(id).oninput = draw; });
replayButton.onclick = replayMission;
svg.onmousemove = event => {
  let point = svg.createSVGPoint();
  point.x = event.clientX; point.y = event.clientY;
  point = point.matrixTransform(svg.getScreenCTM().inverse());
  input('coords').textContent = `ENU 东向 ${point.x.toFixed(1)}，北向 ${(-point.y).toFixed(1)}`;
};

planButton.onclick = async () => {
  input('error').style.display = 'none';
  planButton.disabled = true;
  planButton.textContent = '正在规划…';
  summaryBox.innerHTML = '<h2>规划结果</h2><p>正在计算覆盖路线…</p>';
  try {
    const altitude = +input('alt').value;
    const payload = {
      search_geometry: mapData.search_area,
      flight_altitude_m: altitude,
      home_x_m: +input('home-x').value,
      home_y_m: +input('home-y').value,
      horizontal_clearance_m: +input('hc').value,
      vertical_clearance_m: +input('vc').value,
      scan_direction_deg: +input('angle').value,
      scan_pattern: input('pattern').value,
      capture_frequency_hz: +input('frequency').value,
      control_point_spacing_m: +input('control-spacing').value,
      coverage_speed_mps: +input('lane-speed').value,
      connector_speed_mps: +input('connector-speed').value,
      obstacle_speed_mps: +input('obstacle-speed').value,
      return_speed_mps: +input('connector-speed').value,
      camera: { image_width_px: 1920, image_height_px: 1080,
        horizontal_fov_deg: +input('hfov').value, vertical_fov_deg: +input('vfov').value,
        pitch_deg: -90, yaw_mode: 'follow_path', forward_overlap: +input('fo').value,
        side_overlap: +input('so').value },
    };
    const response = await fetch('/api/plan', {
      method: 'POST', headers: { 'content-type': 'application/json' }, body: JSON.stringify(payload),
    });
    const body = await response.json();
    if (!response.ok) throw new Error(`规划参数或搜索区域无效（状态码 ${response.status}）`);
    planData = body;
    animationState = null;
    const result = body.summary;
    const comparison = result.strategy_comparison.map(item => `${strategyNames[item.pattern] ?? item.pattern}：覆盖率 ${(item.coverage_ratio * 100).toFixed(2)}%，航程 ${item.path_length_m.toFixed(0)} 米`).join('<br>');
    summaryBox.innerHTML = `<h2>规划结果</h2><p>采用策略：${strategyNames[result.scan_pattern] ?? result.scan_pattern}<br>覆盖率：${(result.coverage_ratio * 100).toFixed(2)}%<br>总航程：${result.path_length_m.toFixed(0)} 米<br>覆盖航线：${result.lane_count} 条<br>飞行控制点：${result.flight_waypoint_count} 个<br>预计采图：${result.sampled_image_count} 张<br>未覆盖网格：${result.unreachable.length} 个</p><p><strong>策略比较</strong><br>${comparison}</p>`;
    exportBox.innerHTML = ['flight_plan.json','flight_plan.yaml','waypoints.json','waypoints.csv','patches.geojson','route.geojson','coverage_report.json','visualization.png'].map(file => `<a href="/api/export/${file}">${exportNames[file]}</a>`).join('');
    draw();
    replayButton.disabled = false;
    replayMission();
  } catch (error) {
    summaryBox.innerHTML = '<h2>规划结果</h2><p>规划失败，请查看错误提示。</p>';
    showError(error instanceof Error && error.message.startsWith('规划')
      ? error : new Error('无法连接规划服务，请确认服务正在运行。'));
  } finally {
    planButton.disabled = false;
    planButton.textContent = '开始规划';
  }
};

async function load() {
  try {
    const response = await fetch('/api/map');
    if (!response.ok) throw new Error(`地图加载失败（状态码 ${response.status}）`);
    mapData = await response.json();
    draw();
  } catch (error) {
    showError(new Error('地图加载失败，请刷新页面或确认服务正在运行。'));
  }
}
load();
