import argparse
import json
import os
from http.server import BaseHTTPRequestHandler
from http.server import ThreadingHTTPServer
from urllib.parse import parse_qs
from urllib.parse import urlparse


DEFAULT_MAP_DIR = '/home/nvidia/857_Final_Project_Code/maps'
LEGACY_LABELS_FILE_NAME = 'map_labels.yaml'


VIEWER_HTML = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>QBot Map 3D Viewer</title>
  <style>
    :root {
      color-scheme: dark;
      font-family: Arial, Helvetica, sans-serif;
      background: #111318;
      color: #eef2f6;
    }
    body {
      margin: 0;
      overflow: hidden;
      background: #111318;
    }
    header {
      position: fixed;
      top: 0;
      left: 0;
      right: 0;
      height: 56px;
      display: flex;
      align-items: center;
      gap: 14px;
      padding: 0 16px;
      background: rgba(17, 19, 24, 0.92);
      border-bottom: 1px solid #2a3038;
      z-index: 2;
    }
    h1 {
      font-size: 16px;
      font-weight: 700;
      margin: 0;
      white-space: nowrap;
    }
    .status {
      min-width: 0;
      color: #aab4c0;
      font-size: 13px;
      overflow: hidden;
      text-overflow: ellipsis;
      white-space: nowrap;
    }
    .controls {
      margin-left: auto;
      display: flex;
      align-items: center;
      gap: 12px;
      font-size: 12px;
      color: #c8d0da;
    }
    label {
      display: flex;
      align-items: center;
      gap: 6px;
      white-space: nowrap;
    }
    input[type="range"] {
      width: 104px;
    }
    button {
      border: 1px solid #3a4450;
      background: #202732;
      color: #eef2f6;
      border-radius: 6px;
      padding: 7px 10px;
      cursor: pointer;
    }
    button:hover {
      background: #2a3340;
    }
    button.active {
      border-color: #33c59d;
      background: #163f36;
      color: #dffcf4;
    }
    .label-panel {
      position: fixed;
      left: 14px;
      bottom: 14px;
      width: 260px;
      max-height: 34vh;
      overflow: auto;
      background: rgba(17, 19, 24, 0.9);
      border: 1px solid #2a3038;
      border-radius: 8px;
      padding: 10px;
      z-index: 2;
      font-size: 12px;
    }
    .label-panel h2 {
      font-size: 13px;
      margin: 0 0 8px 0;
    }
    .label-row {
      display: grid;
      grid-template-columns: 1fr auto;
      gap: 8px;
      align-items: center;
      padding: 5px 0;
      border-top: 1px solid #252b33;
    }
    .label-row:first-of-type {
      border-top: 0;
    }
    .label-name {
      color: #eef2f6;
      overflow: hidden;
      text-overflow: ellipsis;
      white-space: nowrap;
    }
    .label-pose {
      color: #aab4c0;
      font-variant-numeric: tabular-nums;
    }
    canvas {
      display: block;
      width: 100vw;
      height: 100vh;
      background: radial-gradient(circle at 50% 45%, #20242b, #0d0f13 72%);
    }
  </style>
</head>
<body>
  <header>
    <h1>QBot Map 3D Viewer</h1>
    <div class="status" id="status">Loading map...</div>
    <div class="controls">
      <label>Height <input id="height" type="range" min="4" max="70" value="28"></label>
      <span>Drag to rotate, wheel to zoom, right-drag to pan</span>
      <button id="labelMode">Add Label</button>
      <button id="reload">Reload</button>
    </div>
  </header>
  <section class="label-panel">
    <h2>Map Labels</h2>
    <div id="labels"></div>
  </section>
  <canvas id="view"></canvas>
  <script>
    const canvas = document.getElementById('view');
    const ctx = canvas.getContext('2d');
    const statusEl = document.getElementById('status');
    const heightEl = document.getElementById('height');
    const reloadEl = document.getElementById('reload');
    const labelModeEl = document.getElementById('labelMode');
    const labelsEl = document.getElementById('labels');
    let mapData = null;
    let labels = {};
    let labelMode = false;
    let projectedCells = [];
    const camera = {
      yaw: -0.65,
      pitch: 0.86,
      zoom: 1.0,
      panX: 0,
      panY: 0,
      dragging: false,
      mode: 'rotate',
      lastX: 0,
      lastY: 0,
      moved: false
    };

    function resize() {
      const ratio = window.devicePixelRatio || 1;
      canvas.width = Math.floor(window.innerWidth * ratio);
      canvas.height = Math.floor(window.innerHeight * ratio);
      ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
      draw();
    }

    async function loadMap() {
      statusEl.textContent = 'Loading map...';
      const response = await fetch('/api/map?max_size=170', {cache: 'no-store'});
      if (!response.ok) {
        statusEl.textContent = await response.text();
        mapData = null;
        draw();
        return;
      }
      mapData = await response.json();
      statusEl.textContent = `${mapData.name} (${mapData.width} x ${mapData.height}, sampled ${mapData.sampled_width} x ${mapData.sampled_height})`;
      await loadLabels();
      draw();
    }

    async function loadLabels() {
      const response = await fetch(`/api/labels?map=${encodeURIComponent(mapData.name)}`, {cache: 'no-store'});
      if (!response.ok) {
        labels = {};
        renderLabels();
        return;
      }
      const payload = await response.json();
      labels = payload.locations || {};
      renderLabels();
    }

    async function saveLabel(name, pose) {
      const response = await fetch('/api/labels', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({map: mapData.name, name, pose})
      });
      if (!response.ok) {
        statusEl.textContent = await response.text();
        return;
      }
      const payload = await response.json();
      labels = payload.locations || {};
      renderLabels();
      draw();
      statusEl.textContent = `Saved label: ${name}`;
    }

    function renderLabels() {
      const names = Object.keys(labels).sort();
      if (names.length === 0) {
        labelsEl.innerHTML = '<div class="label-pose">No labels yet.</div>';
        return;
      }
      labelsEl.innerHTML = names.map((name) => {
        const pose = labels[name];
        return `<div class="label-row"><div class="label-name">${escapeHtml(name)}</div><div class="label-pose">${pose.x.toFixed(2)}, ${pose.y.toFixed(2)}</div></div>`;
      }).join('');
    }

    function escapeHtml(value) {
      return String(value).replace(/[&<>"']/g, (char) => ({
        '&': '&amp;',
        '<': '&lt;',
        '>': '&gt;',
        '"': '&quot;',
        "'": '&#39;'
      }[char]));
    }

    function cellStyle(value) {
      if (value < 0) {
        return {height: 0.35, fill: '#59616d', top: '#737d8a'};
      }
      if (value >= 65) {
        return {height: 1.0, fill: '#28313c', top: '#dfe7ef'};
      }
      if (value <= 25) {
        return {height: 0.05, fill: '#1f8f72', top: '#33c59d'};
      }
      return {height: 0.25, fill: '#657080', top: '#8893a1'};
    }

    function project(x, y, z, config) {
      const [cosYaw, sinYaw, cosPitch, sinPitch, scale, centerX, centerY] = config;
      const rx = x * cosYaw - y * sinYaw;
      const ry = x * sinYaw + y * cosYaw;
      const rz = z;
      const py = ry * cosPitch - rz * sinPitch;
      return [
        centerX + camera.panX + rx * scale,
        centerY + camera.panY + py * scale
      ];
    }

    function drawBlock(cx, cy, size, height, style, config) {
      const half = size / 2;
      const bottom = [
        project(cx - half, cy - half, 0, config),
        project(cx + half, cy - half, 0, config),
        project(cx + half, cy + half, 0, config),
        project(cx - half, cy + half, 0, config)
      ];
      const top = [
        project(cx - half, cy - half, height, config),
        project(cx + half, cy - half, height, config),
        project(cx + half, cy + half, height, config),
        project(cx - half, cy + half, height, config)
      ];

      ctx.beginPath();
      ctx.moveTo(bottom[1][0], bottom[1][1]);
      ctx.lineTo(bottom[2][0], bottom[2][1]);
      ctx.lineTo(top[2][0], top[2][1]);
      ctx.lineTo(top[1][0], top[1][1]);
      ctx.closePath();
      ctx.fillStyle = style.fill;
      ctx.fill();

      ctx.beginPath();
      ctx.moveTo(bottom[2][0], bottom[2][1]);
      ctx.lineTo(bottom[3][0], bottom[3][1]);
      ctx.lineTo(top[3][0], top[3][1]);
      ctx.lineTo(top[2][0], top[2][1]);
      ctx.closePath();
      ctx.fillStyle = '#171d24';
      ctx.globalAlpha = 0.72;
      ctx.fill();
      ctx.globalAlpha = 1;

      ctx.beginPath();
      ctx.moveTo(top[0][0], top[0][1]);
      ctx.lineTo(top[1][0], top[1][1]);
      ctx.lineTo(top[2][0], top[2][1]);
      ctx.lineTo(top[3][0], top[3][1]);
      ctx.closePath();
      ctx.fillStyle = style.top;
      ctx.fill();
    }

    function mapPoseForCell(cell, sampledX, sampledY) {
      const stride = mapData.stride || 1;
      const pixelX = sampledX * stride;
      const pixelY = sampledY * stride;
      const origin = mapData.origin_values || [0, 0, 0];
      return {
        x: origin[0] + pixelX * mapData.resolution,
        y: origin[1] + (mapData.height - pixelY) * mapData.resolution,
        yaw: 0.0
      };
    }

    function projectMapPose(pose, config) {
      const origin = mapData.origin_values || [0, 0, 0];
      const pixelX = (pose.x - origin[0]) / mapData.resolution;
      const pixelY = mapData.height - ((pose.y - origin[1]) / mapData.resolution);
      const sampledX = pixelX / (mapData.stride || 1);
      const sampledY = pixelY / (mapData.stride || 1);
      const centeredX = sampledX - mapData.sampled_width / 2;
      const centeredY = sampledY - mapData.sampled_height / 2;
      return project(centeredX, centeredY, 4.2, config);
    }

    function drawLabels(config) {
      const names = Object.keys(labels).sort();
      for (const name of names) {
        const pose = labels[name];
        const [x, y] = projectMapPose(pose, config);
        ctx.beginPath();
        ctx.arc(x, y, 6, 0, Math.PI * 2);
        ctx.fillStyle = '#ffcf5a';
        ctx.fill();
        ctx.lineWidth = 2;
        ctx.strokeStyle = '#111318';
        ctx.stroke();
        ctx.font = '12px Arial, Helvetica, sans-serif';
        ctx.textBaseline = 'middle';
        const label = name.length > 24 ? `${name.slice(0, 23)}...` : name;
        const textWidth = ctx.measureText(label).width;
        ctx.fillStyle = 'rgba(17, 19, 24, 0.82)';
        ctx.fillRect(x + 9, y - 11, textWidth + 10, 22);
        ctx.fillStyle = '#eef2f6';
        ctx.fillText(label, x + 14, y);
      }
    }

    function draw() {
      const width = window.innerWidth;
      const height = window.innerHeight;
      ctx.clearRect(0, 0, width, height);
      if (!mapData) {
        return;
      }

      const w = mapData.sampled_width;
      const h = mapData.sampled_height;
      const values = mapData.data;
      const heightScale = Number(heightEl.value) / 10;
      const cosYaw = Math.cos(camera.yaw);
      const sinYaw = Math.sin(camera.yaw);
      const cosPitch = Math.cos(camera.pitch);
      const sinPitch = Math.sin(camera.pitch);
      const scale = Math.min((width - 80) / (w + h), (height - 120) / (w + h)) * 2.0 * camera.zoom;
      const centerX = width / 2;
      const centerY = height / 2 + 95;
      const config = [cosYaw, sinYaw, cosPitch, sinPitch, scale, centerX, centerY];

      const cells = [];
      projectedCells = [];
      for (let y = 0; y < h; y++) {
        for (let x = 0; x < w; x++) {
          const centeredX = x - w / 2;
          const centeredY = y - h / 2;
          const depth = centeredX * sinYaw + centeredY * cosYaw;
          cells.push({
            x: centeredX,
            y: centeredY,
            sampledX: x,
            sampledY: y,
            value: values[y * w + x],
            depth
          });
        }
      }
      cells.sort((a, b) => a.depth - b.depth);

      for (const cell of cells) {
        const style = cellStyle(cell.value);
        drawBlock(cell.x, cell.y, 0.92, style.height * heightScale, style, config);
        const [screenX, screenY] = project(cell.x, cell.y, style.height * heightScale + 0.35, config);
        projectedCells.push({
          screenX,
          screenY,
          pose: mapPoseForCell(cell, cell.sampledX, cell.sampledY)
        });
      }
      drawLabels(config);
    }

    function clamp(value, min, max) {
      return Math.max(min, Math.min(max, value));
    }

    canvas.addEventListener('contextmenu', (event) => event.preventDefault());
    canvas.addEventListener('pointerdown', (event) => {
      camera.dragging = true;
      camera.mode = event.button === 2 ? 'pan' : 'rotate';
      camera.lastX = event.clientX;
      camera.lastY = event.clientY;
      camera.moved = false;
      canvas.setPointerCapture(event.pointerId);
    });
    canvas.addEventListener('pointermove', (event) => {
      if (!camera.dragging) {
        return;
      }
      const dx = event.clientX - camera.lastX;
      const dy = event.clientY - camera.lastY;
      camera.lastX = event.clientX;
      camera.lastY = event.clientY;
      if (Math.abs(dx) + Math.abs(dy) > 3) {
        camera.moved = true;
      }

      if (camera.mode === 'pan') {
        camera.panX += dx;
        camera.panY += dy;
      } else {
        camera.yaw += dx * 0.012;
        camera.pitch = clamp(camera.pitch + dy * 0.008, 0.28, 1.42);
      }
      draw();
    });
    canvas.addEventListener('pointerup', (event) => {
      const wasClick = !camera.moved && event.button !== 2;
      camera.dragging = false;
      canvas.releasePointerCapture(event.pointerId);
      if (wasClick && labelMode) {
        addLabelAt(event.clientX, event.clientY);
      }
    });
    canvas.addEventListener('pointercancel', () => {
      camera.dragging = false;
    });
    canvas.addEventListener('wheel', (event) => {
      event.preventDefault();
      const direction = event.deltaY > 0 ? 0.9 : 1.1;
      camera.zoom = clamp(camera.zoom * direction, 0.35, 5.0);
      draw();
    }, {passive: false});

    window.addEventListener('resize', resize);
    heightEl.addEventListener('input', draw);
    reloadEl.addEventListener('click', loadMap);
    labelModeEl.addEventListener('click', () => {
      labelMode = !labelMode;
      labelModeEl.classList.toggle('active', labelMode);
      statusEl.textContent = labelMode ? 'Label mode: click the map to add a label.' : 'Label mode off.';
    });

    function nearestCell(clientX, clientY) {
      let best = null;
      let bestDistance = Infinity;
      for (const cell of projectedCells) {
        const dx = cell.screenX - clientX;
        const dy = cell.screenY - clientY;
        const distance = dx * dx + dy * dy;
        if (distance < bestDistance) {
          best = cell;
          bestDistance = distance;
        }
      }
      return bestDistance <= 1600 ? best : null;
    }

    function addLabelAt(clientX, clientY) {
      const cell = nearestCell(clientX, clientY);
      if (!cell) {
        statusEl.textContent = 'Click closer to the map surface to place a label.';
        return;
      }
      const name = window.prompt('Label name, for example room_101 or entrance:');
      if (!name) {
        return;
      }
      const cleanName = name.trim().replace(/[^A-Za-z0-9_-]+/g, '_');
      if (!cleanName) {
        statusEl.textContent = 'Label name was empty.';
        return;
      }
      saveLabel(cleanName, cell.pose);
    }

    resize();
    loadMap();
  </script>
</body>
</html>
"""


def find_latest_map(map_dir):
    if not os.path.isdir(map_dir):
        return None

    yaml_paths = [
        os.path.join(map_dir, name)
        for name in os.listdir(map_dir)
        if name.endswith('.yaml')
        and not name.endswith('.labels.yaml')
        and name != LEGACY_LABELS_FILE_NAME
    ]
    if not yaml_paths:
        return None

    return max(yaml_paths, key=os.path.getmtime)


def parse_map_yaml(yaml_path):
    fields = {}
    with open(yaml_path, 'r', encoding='utf-8') as yaml_file:
        for line in yaml_file:
            if ':' not in line:
                continue
            key, value = line.split(':', 1)
            fields[key.strip()] = value.strip()

    image_name = fields.get('image')
    if not image_name:
        raise ValueError(f'Map yaml does not contain an image field: {yaml_path}')

    image_path = image_name
    if not os.path.isabs(image_path):
        image_path = os.path.join(os.path.dirname(yaml_path), image_path)

    return {
        'yaml_path': yaml_path,
        'image_path': image_path,
        'resolution': float(fields.get('resolution', '0.05')),
        'origin': fields.get('origin', '[0, 0, 0]'),
        'origin_values': parse_origin(fields.get('origin', '[0, 0, 0]')),
    }


def parse_origin(origin_text):
    cleaned = origin_text.strip().strip('[]')
    values = []
    for part in cleaned.split(','):
        try:
            values.append(float(part.strip()))
        except ValueError:
            values.append(0.0)
    while len(values) < 3:
        values.append(0.0)
    return values[:3]


def read_pgm(path):
    with open(path, 'rb') as pgm_file:
        magic = pgm_file.readline().strip()
        if magic != b'P5':
            raise ValueError(f'Only binary PGM P5 maps are supported: {path}')

        line = pgm_file.readline()
        while line.startswith(b'#'):
            line = pgm_file.readline()

        width, height = [int(part) for part in line.split()]
        max_value = int(pgm_file.readline().strip())
        if max_value <= 0 or max_value > 255:
            raise ValueError(f'Unsupported PGM max value {max_value}: {path}')

        pixels = pgm_file.read(width * height)
        if len(pixels) != width * height:
            raise ValueError(f'PGM data is incomplete: {path}')

    return width, height, pixels


def pixel_to_occupancy(pixel):
    if pixel == 205:
        return -1
    if pixel <= 25:
        return 100
    if pixel >= 250:
        return 0
    return int(round((255 - pixel) * 100 / 255))


def load_sampled_map(map_dir, max_size):
    yaml_path = find_latest_map(map_dir)
    if yaml_path is None:
        raise FileNotFoundError(f'No .yaml maps found in {map_dir}')

    metadata = parse_map_yaml(yaml_path)
    width, height, pixels = read_pgm(metadata['image_path'])
    stride = max(1, int(max(width, height) / max_size))
    sampled_width = (width + stride - 1) // stride
    sampled_height = (height + stride - 1) // stride
    sampled = []

    for y in range(0, height, stride):
        for x in range(0, width, stride):
            sampled.append(pixel_to_occupancy(pixels[y * width + x]))

    return {
        'name': os.path.basename(yaml_path),
        'width': width,
        'height': height,
        'sampled_width': sampled_width,
        'sampled_height': sampled_height,
        'stride': stride,
        'resolution': metadata['resolution'],
        'origin': metadata['origin'],
        'origin_values': metadata['origin_values'],
        'data': sampled,
    }


def clean_map_name(map_name):
    basename = os.path.basename(str(map_name))
    if not basename.endswith('.yaml'):
        raise ValueError('Map name must be a .yaml file')
    if basename.endswith('.labels.yaml') or basename == LEGACY_LABELS_FILE_NAME:
        raise ValueError('Invalid map name for labels')
    return basename


def labels_path(map_dir, map_name):
    map_basename = clean_map_name(map_name)
    label_name = f'{os.path.splitext(map_basename)[0]}.labels.yaml'
    return os.path.join(map_dir, label_name)


def load_labels(map_dir, map_name):
    path = labels_path(map_dir, map_name)
    if not os.path.exists(path):
        return {
            'map': clean_map_name(map_name),
            'labels_file': os.path.basename(path),
            'locations': {},
        }

    labels = {}
    current_name = None
    with open(path, 'r', encoding='utf-8') as label_file:
        for raw_line in label_file:
            line = raw_line.rstrip()
            stripped = line.strip()
            if not stripped or stripped.startswith('#'):
                continue
            if line.startswith('  ') and stripped.endswith(':'):
                current_name = stripped[:-1]
                labels[current_name] = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
                continue
            if line.startswith('    ') and current_name and ':' in stripped:
                key, value = stripped.split(':', 1)
                if key in ('x', 'y', 'yaw'):
                    labels[current_name][key] = float(value.strip())

    return {
        'map': clean_map_name(map_name),
        'labels_file': os.path.basename(path),
        'locations': labels,
    }


def write_labels(map_dir, map_name, labels):
    os.makedirs(map_dir, exist_ok=True)
    path = labels_path(map_dir, map_name)
    lines = [
        f'# Named navigation targets for {clean_map_name(map_name)}.',
        f'map: {clean_map_name(map_name)}',
        '',
        'locations:',
    ]
    for name in sorted(labels):
        pose = labels[name]
        lines.extend([
            f'  {name}:',
            f'    x: {float(pose["x"]):.6f}',
            f'    y: {float(pose["y"]):.6f}',
            f'    yaw: {float(pose.get("yaw", 0.0)):.6f}',
        ])

    with open(path, 'w', encoding='utf-8') as label_file:
        label_file.write('\n'.join(lines) + '\n')


def clean_label_name(name):
    cleaned = ''.join(
        char if char.isalnum() or char in ('_', '-') else '_'
        for char in str(name).strip()
    )
    return cleaned.strip('_')


class MapViewerHandler(BaseHTTPRequestHandler):
    map_dir = DEFAULT_MAP_DIR

    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path in ('/', '/index.html'):
            self.send_text(VIEWER_HTML, 'text/html; charset=utf-8')
            return

        if parsed.path == '/api/map':
            query = parse_qs(parsed.query)
            max_size = int(query.get('max_size', ['170'])[0])
            try:
                payload = load_sampled_map(self.map_dir, max_size)
            except (OSError, ValueError) as exc:
                self.send_error(404, str(exc))
                return

            self.send_json(payload)
            return

        if parsed.path == '/api/labels':
            query = parse_qs(parsed.query)
            map_name = query.get('map', [''])[0]
            try:
                self.send_json(load_labels(self.map_dir, map_name))
            except ValueError as exc:
                self.send_error(400, str(exc))
            return

        self.send_error(404, 'Not found')

    def do_POST(self):
        parsed = urlparse(self.path)
        if parsed.path != '/api/labels':
            self.send_error(404, 'Not found')
            return

        length = int(self.headers.get('Content-Length', '0'))
        try:
            payload = json.loads(self.rfile.read(length).decode('utf-8'))
        except json.JSONDecodeError:
            self.send_error(400, 'Invalid JSON')
            return

        name = clean_label_name(payload.get('name', ''))
        map_name = payload.get('map', '')
        pose = payload.get('pose', {})
        if not name:
            self.send_error(400, 'Label name is required')
            return

        try:
            labels = load_labels(self.map_dir, map_name)['locations']
            labels[name] = {
                'x': float(pose['x']),
                'y': float(pose['y']),
                'yaw': float(pose.get('yaw', 0.0)),
            }
            write_labels(self.map_dir, map_name, labels)
        except (KeyError, TypeError, ValueError) as exc:
            self.send_error(400, f'Invalid label pose: {exc}')
            return

        self.send_json(load_labels(self.map_dir, map_name))

    def log_message(self, format_string, *args):
        print(f'[map_3d_viewer] {self.address_string()} - {format_string % args}')

    def send_text(self, text, content_type):
        body = text.encode('utf-8')
        self.send_response(200)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def send_json(self, payload):
        body = json.dumps(payload).encode('utf-8')
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Cache-Control', 'no-store')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)


def main():
    parser = argparse.ArgumentParser(description='Stream a browser-based 3D map viewer.')
    parser.add_argument('--map-dir', default=DEFAULT_MAP_DIR)
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8090)
    args = parser.parse_args()

    handler = MapViewerHandler
    handler.map_dir = os.path.abspath(os.path.expanduser(args.map_dir))
    server = ThreadingHTTPServer((args.host, args.port), handler)

    print(f'Serving 3D map viewer at http://{args.host}:{args.port}')
    print(f'Loading latest map from: {handler.map_dir}')
    print('Over SSH, use: ssh -L 8090:localhost:8090 nvidia@<qbot-ip>')
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == '__main__':
    main()
