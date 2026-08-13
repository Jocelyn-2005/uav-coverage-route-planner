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
    title.textContent = `${building.id} · ${building.height_m.toFixed(1)} m`;
    rectangle.append(title);
    svg.append(rectangle);
  });
  if (planData && layer('patches')) planData.patches.forEach(patch =>
    rings(patch.geometry).forEach(ring => svg.append(element('polygon', {
      points: polygonPoints(ring),
    }, `patch ${patch.covered ? '' : 'uncovered'}`))));
  if (planData && layer('route')) svg.append(element('polyline', {
    points: planData.waypoints.map(waypoint => `${waypoint.x},${-waypoint.y}`).join(' '),
  }, 'route'));
  if (planData && layer('waypoints')) planData.waypoints.forEach(waypoint => {
    const circle = element('circle', {
      cx: waypoint.x, cy: -waypoint.y, r: waypoint.capture ? .8 : 1.1,
    }, `waypoint ${waypoint.capture ? '' : 'transit'}`);
    const title = element('title', {});
    title.textContent = `${waypoint.id} · ENU ${waypoint.x.toFixed(2)}, ${waypoint.y.toFixed(2)}, ${waypoint.z.toFixed(2)} · ${waypoint.kind}`;
    circle.append(title);
    svg.append(circle);
  });
  if (planData) {
    svg.append(element('circle', { cx: planData.home[0], cy: -planData.home[1], r: 2 }, 'home'));
    const title = element('title', {}); title.textContent = `HOME · ENU ${planData.home[0]}, ${planData.home[1]}`;
    svg.lastChild.append(title);
  }
  drawAnimationState();
}

function drawAnimationState() {
  if (!animationState || !planData) return;
  animationState.captured.forEach(waypoint => {
    if (!waypoint.camera_footprint_enu) return;
    rings(waypoint.camera_footprint_enu).forEach(ring => svg.append(element('polygon', {
      points: polygonPoints(ring),
    }, 'captured-footprint')));
  });
  if (animationState.travelled.length > 1) svg.append(element('polyline', {
    points: animationState.travelled.map(point => `${point[0]},${-point[1]}`).join(' '),
  }, 'travelled'));
  if (animationState.position) svg.append(element('circle', {
    cx: animationState.position[0], cy: -animationState.position[1], r: 1.7,
  }, 'drone'));
}

function replayMission() {
  if (!planData || planData.waypoints.length < 2) return;
  cancelAnimationFrame(animationFrame);
  const route = planData.waypoints;
  animationState = { segment: 0, progress: 0, captured: [], travelled: [[route[0].x, route[0].y]],
    position: [route[0].x, route[0].y], lastTime: null };
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
    if (ratio === 1) {
      animationState.travelled.push([to.x, to.y]);
      if (to.capture) animationState.captured.push(to);
      animationState.segment += 1;
      animationState.progress = 0;
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
replayButton.onclick = replayMission;
svg.onmousemove = event => {
  let point = svg.createSVGPoint();
  point.x = event.clientX; point.y = event.clientY;
  point = point.matrixTransform(svg.getScreenCTM().inverse());
  input('coords').textContent = `ENU x ${point.x.toFixed(1)}, y ${(-point.y).toFixed(1)}`;
};

planButton.onclick = async () => {
  input('error').style.display = 'none';
  planButton.disabled = true;
  planButton.textContent = 'Planning...';
  summaryBox.innerHTML = '<h2>Result</h2><p>Planning route...</p>';
  try {
    const altitude = +input('alt').value;
    const payload = {
      search_geometry: mapData.search_area,
      flight_altitude_m: altitude,
      horizontal_clearance_m: +input('hc').value,
      vertical_clearance_m: +input('vc').value,
      scan_direction_deg: +input('angle').value,
      scan_pattern: input('pattern').value,
      camera: { image_width_px: 1920, image_height_px: 1080,
        horizontal_fov_deg: +input('hfov').value, vertical_fov_deg: +input('vfov').value,
        pitch_deg: -90, yaw_mode: 'follow_path', forward_overlap: +input('fo').value,
        side_overlap: +input('so').value },
    };
    const response = await fetch('/api/plan', {
      method: 'POST', headers: { 'content-type': 'application/json' }, body: JSON.stringify(payload),
    });
    const body = await response.json();
    if (!response.ok) throw new Error(body.detail || `Planning failed (${response.status})`);
    planData = body;
    animationState = null;
    const result = body.summary;
    const comparison = result.strategy_comparison.map(item => `${item.pattern}: ${(item.coverage_ratio * 100).toFixed(2)}%, ${item.path_length_m.toFixed(0)} m`).join('<br>');
    summaryBox.innerHTML = `<h2>Result</h2><p>Strategy ${result.scan_pattern}<br>Coverage ${(result.coverage_ratio * 100).toFixed(2)}%<br>Path ${result.path_length_m.toFixed(0)} m<br>Capture ${result.capture_count} · Transit ${result.transit_count}<br>Unreachable ${result.unreachable.length}</p><p>${comparison}</p>`;
    exportBox.innerHTML = ['waypoints.json','waypoints.csv','patches.geojson','route.geojson','coverage_report.json','visualization.png'].map(file => `<a href="/api/export/${file}">${file}</a>`).join('');
    draw();
    replayButton.disabled = false;
    replayMission();
  } catch (error) {
    summaryBox.innerHTML = '<h2>Result</h2><p>Planning failed</p>';
    showError(error);
  } finally {
    planButton.disabled = false;
    planButton.textContent = 'Plan';
  }
};

async function load() {
  try {
    const response = await fetch('/api/map');
    if (!response.ok) throw new Error(`Map request failed (${response.status})`);
    mapData = await response.json();
    draw();
  } catch (error) {
    showError(error);
  }
}
load();
