import json
import math
import os
from http.server import BaseHTTPRequestHandler
from http.server import ThreadingHTTPServer
from threading import Lock
from threading import Thread

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


DEFAULT_MASK_FILE = (
    '/home/nvidia/857_Final_Project_Code/maps/qbot_lidar_filter.json'
)


VIEWER_HTML = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>QBot LiDAR Filter Editor</title>
  <style>
    body {
      margin: 0;
      background: #101318;
      color: #edf3f8;
      font-family: Arial, Helvetica, sans-serif;
      overflow: hidden;
    }
    header {
      height: 56px;
      display: flex;
      align-items: center;
      gap: 10px;
      padding: 0 14px;
      background: #181d24;
      border-bottom: 1px solid #2a333d;
      box-sizing: border-box;
    }
    h1 {
      margin: 0;
      font-size: 16px;
      white-space: nowrap;
    }
    button {
      border: 1px solid #3a4450;
      background: #202732;
      color: #eef2f6;
      border-radius: 6px;
      padding: 7px 10px;
      cursor: pointer;
    }
    button.active {
      border-color: #33c59d;
      background: #164036;
    }
    button.danger {
      border-color: #7f3b45;
      background: #3b1f26;
    }
    #status {
      margin-left: auto;
      color: #b8c3cf;
      font-size: 13px;
      white-space: nowrap;
      overflow: hidden;
      text-overflow: ellipsis;
    }
    canvas {
      display: block;
      width: 100vw;
      height: calc(100vh - 56px);
      background: #0b0e13;
    }
  </style>
</head>
<body>
  <header>
    <h1>QBot LiDAR Filter</h1>
    <button id="drawMode">Draw Ignore</button>
    <button id="undoPoint">Undo Point</button>
    <button id="saveRegion">Save Region</button>
    <button id="clearRegions" class="danger">Clear Shapes</button>
    <div id="status">Waiting...</div>
  </header>
  <canvas id="view"></canvas>
  <script>
    const canvas = document.getElementById('view');
    const ctx = canvas.getContext('2d');
    const statusEl = document.getElementById('status');
    const drawModeEl = document.getElementById('drawMode');
    const undoPointEl = document.getElementById('undoPoint');
    const saveRegionEl = document.getElementById('saveRegion');
    const clearRegionsEl = document.getElementById('clearRegions');
    let state = null;
    let drawMode = false;
    let draft = [];
    let viewConfig = null;

    function resize() {
      const ratio = window.devicePixelRatio || 1;
      canvas.width = Math.floor(window.innerWidth * ratio);
      canvas.height = Math.floor((window.innerHeight - 56) * ratio);
      ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
      draw();
    }

    function scans() {
      return state || {raw_scan: null, filtered_scan: null, mask: {regions: []}};
    }

    function maxRange() {
      const current = scans();
      const rawMax = current.raw_scan ? current.raw_scan.range_max : 6;
      const filteredMax = current.filtered_scan
        ? current.filtered_scan.range_max
        : rawMax;
      return Math.min(Math.max(rawMax, filteredMax, 1), 6);
    }

    function makeViewConfig() {
      const width = canvas.clientWidth;
      const height = canvas.clientHeight;
      const range = maxRange();
      return {
        cx: width / 2,
        cy: height / 2,
        scale: Math.min(width, height) * 0.45 / range,
        range
      };
    }

    function worldToScreen(point) {
      return {
        x: viewConfig.cx + point.x * viewConfig.scale,
        y: viewConfig.cy - point.y * viewConfig.scale
      };
    }

    function screenToWorld(clientX, clientY) {
      const rect = canvas.getBoundingClientRect();
      return {
        x: (clientX - rect.left - viewConfig.cx) / viewConfig.scale,
        y: (viewConfig.cy - (clientY - rect.top)) / viewConfig.scale
      };
    }

    function drawGrid() {
      ctx.strokeStyle = '#26313d';
      ctx.lineWidth = 1;
      for (let r = 1; r <= viewConfig.range; r += 1) {
        ctx.beginPath();
        ctx.arc(
          viewConfig.cx,
          viewConfig.cy,
          r * viewConfig.scale,
          0,
          Math.PI * 2
        );
        ctx.stroke();
      }
      ctx.strokeStyle = '#56616f';
      ctx.beginPath();
      ctx.moveTo(viewConfig.cx - viewConfig.range * viewConfig.scale, viewConfig.cy);
      ctx.lineTo(viewConfig.cx + viewConfig.range * viewConfig.scale, viewConfig.cy);
      ctx.moveTo(viewConfig.cx, viewConfig.cy - viewConfig.range * viewConfig.scale);
      ctx.lineTo(viewConfig.cx, viewConfig.cy + viewConfig.range * viewConfig.scale);
      ctx.stroke();
    }

    function drawRobot() {
      ctx.fillStyle = '#3aa7ff';
      ctx.beginPath();
      ctx.arc(viewConfig.cx, viewConfig.cy, 9, 0, Math.PI * 2);
      ctx.fill();
      ctx.strokeStyle = '#3aa7ff';
      ctx.lineWidth = 4;
      ctx.lineCap = 'round';
      ctx.beginPath();
      ctx.moveTo(viewConfig.cx, viewConfig.cy);
      ctx.lineTo(viewConfig.cx, viewConfig.cy + 0.6 * viewConfig.scale);
      ctx.stroke();
      ctx.beginPath();
      ctx.moveTo(viewConfig.cx, viewConfig.cy + 0.72 * viewConfig.scale);
      ctx.lineTo(viewConfig.cx - 8, viewConfig.cy + 0.5 * viewConfig.scale);
      ctx.lineTo(viewConfig.cx + 8, viewConfig.cy + 0.5 * viewConfig.scale);
      ctx.closePath();
      ctx.fill();
    }

    function drawScan(scan, color, size) {
      if (!scan) {
        return;
      }
      ctx.fillStyle = color;
      for (const point of scan.points) {
        const screen = worldToScreen(point);
        ctx.fillRect(screen.x - size / 2, screen.y - size / 2, size, size);
      }
    }

    function drawPolygon(points, fill, stroke) {
      if (points.length === 0) {
        return;
      }
      ctx.beginPath();
      const first = worldToScreen(points[0]);
      ctx.moveTo(first.x, first.y);
      for (const point of points.slice(1)) {
        const screen = worldToScreen(point);
        ctx.lineTo(screen.x, screen.y);
      }
      if (points.length >= 3) {
        ctx.closePath();
        ctx.fillStyle = fill;
        ctx.fill();
      }
      ctx.strokeStyle = stroke;
      ctx.lineWidth = 2;
      ctx.stroke();
      ctx.fillStyle = stroke;
      for (const point of points) {
        const screen = worldToScreen(point);
        ctx.beginPath();
        ctx.arc(screen.x, screen.y, 4, 0, Math.PI * 2);
        ctx.fill();
      }
    }

    function drawRegions() {
      const regions = scans().mask.regions || [];
      for (const region of regions) {
        drawPolygon(region.points || [], 'rgba(255, 207, 90, 0.16)', '#ffcf5a');
      }
      drawPolygon(draft, 'rgba(51, 197, 157, 0.16)', '#33c59d');
    }

    function drawLegend() {
      ctx.font = '13px Arial';
      ctx.fillStyle = '#ff5555';
      ctx.fillText('raw /scan', 14, 24);
      ctx.fillStyle = '#33c59d';
      ctx.fillText('filtered /scan_slam', 14, 44);
      ctx.fillStyle = '#ffcf5a';
      ctx.fillText('saved ignore shape', 14, 64);
    }

    function draw() {
      const width = canvas.clientWidth;
      const height = canvas.clientHeight;
      ctx.clearRect(0, 0, width, height);
      viewConfig = makeViewConfig();
      drawGrid();
      drawRegions();
      drawScan(scans().raw_scan, '#ff5555', 3);
      drawScan(scans().filtered_scan, '#33c59d', 2);
      drawRobot();
      drawLegend();
      if (!state) {
        ctx.fillStyle = '#b8c3cf';
        ctx.font = '16px Arial';
        ctx.fillText('Waiting for LiDAR...', 18, 92);
      }
    }

    async function poll() {
      try {
        const response = await fetch('/api/state', {cache: 'no-store'});
        if (!response.ok) {
          statusEl.textContent = await response.text();
          state = null;
        } else {
          state = await response.json();
          const raw = state.raw_scan;
          const filtered = state.filtered_scan;
          const regionCount = state.mask.regions.length;
          statusEl.textContent = raw && filtered
            ? `${filtered.valid_points}/${raw.valid_points} kept`
              + ` | ${regionCount} shapes`
            : `Waiting | ${regionCount} shapes`;
        }
      } catch (error) {
        statusEl.textContent = `Disconnected: ${error}`;
        state = null;
      }
      draw();
    }

    async function saveMask(regions) {
      const response = await fetch('/api/mask', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({regions})
      });
      if (!response.ok) {
        statusEl.textContent = await response.text();
        return false;
      }
      state = await response.json();
      draw();
      return true;
    }

    function currentRegions() {
      return (scans().mask.regions || []).map((region) => ({
        name: region.name,
        points: region.points
      }));
    }

    canvas.addEventListener('click', (event) => {
      if (!drawMode) {
        return;
      }
      draft.push(screenToWorld(event.clientX, event.clientY));
      draw();
    });

    drawModeEl.addEventListener('click', () => {
      drawMode = !drawMode;
      drawModeEl.classList.toggle('active', drawMode);
      statusEl.textContent = drawMode
        ? 'Click around the bad LiDAR points, then Save Region.'
        : 'Draw mode off.';
    });

    undoPointEl.addEventListener('click', () => {
      draft.pop();
      draw();
    });

    saveRegionEl.addEventListener('click', async () => {
      if (draft.length < 3) {
        statusEl.textContent = 'Need at least 3 points to save a closed shape.';
        return;
      }
      const regions = currentRegions();
      regions.push({
        name: `ignore_${regions.length + 1}`,
        points: draft
      });
      if (await saveMask(regions)) {
        draft = [];
        statusEl.textContent = 'Saved ignore shape. Filter reloads automatically.';
      }
    });

    clearRegionsEl.addEventListener('click', async () => {
      if (!window.confirm('Clear all drawn LiDAR ignore shapes?')) {
        return;
      }
      draft = [];
      if (await saveMask([])) {
        statusEl.textContent = 'Cleared drawn ignore shapes.';
      }
    });

    window.addEventListener('resize', resize);
    resize();
    poll();
    setInterval(poll, 250);
  </script>
</body>
</html>
"""


class LidarWebViewer(Node):
    def __init__(self):
        super().__init__('lidar_web_viewer')
        self.declare_parameter('raw_scan_topic', '/scan')
        self.declare_parameter('filtered_scan_topic', '/scan_slam')
        self.declare_parameter('mask_file', DEFAULT_MASK_FILE)
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8091)

        self.lock = Lock()
        self.latest_raw_scan = None
        self.latest_filtered_scan = None
        self.mask = {'regions': [], 'beam_indices': []}
        raw_scan_topic = self.get_parameter('raw_scan_topic').value
        filtered_scan_topic = self.get_parameter('filtered_scan_topic').value
        self.mask_file = self.get_parameter('mask_file').value
        host = self.get_parameter('host').value
        port = int(self.get_parameter('port').value)

        self.load_mask()
        self.create_subscription(
            LaserScan,
            raw_scan_topic,
            self.raw_scan_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan,
            filtered_scan_topic,
            self.filtered_scan_callback,
            qos_profile_sensor_data,
        )

        handler = self.make_handler()
        self.server = ThreadingHTTPServer((host, port), handler)
        self.server_thread = Thread(target=self.server.serve_forever, daemon=True)
        self.server_thread.start()
        self.get_logger().info(
            f'Streaming LiDAR filter editor at http://{host}:{port}'
        )
        self.get_logger().info(
            f'Raw scan: {raw_scan_topic}; filtered scan: {filtered_scan_topic}'
        )

    def raw_scan_callback(self, msg):
        with self.lock:
            self.latest_raw_scan = self.scan_to_payload(msg)

    def filtered_scan_callback(self, msg):
        with self.lock:
            self.latest_filtered_scan = self.scan_to_payload(msg)

    def scan_to_payload(self, msg):
        points = []
        for index, value in enumerate(msg.ranges):
            if (
                not math.isfinite(value)
                or value < msg.range_min
                or value > msg.range_max
            ):
                continue
            angle = msg.angle_min + index * msg.angle_increment
            points.append({
                'x': value * math.cos(angle),
                'y': value * math.sin(angle),
            })

        return {
            'range_min': msg.range_min,
            'range_max': msg.range_max,
            'total_ranges': len(msg.ranges),
            'valid_points': len(points),
            'points': points,
        }

    def load_mask(self):
        if not self.mask_file or not os.path.exists(self.mask_file):
            return
        try:
            with open(self.mask_file, 'r', encoding='utf-8') as file:
                self.mask = normalize_mask(json.load(file))
        except (OSError, json.JSONDecodeError) as error:
            self.get_logger().warning(
                f'Could not load LiDAR mask file {self.mask_file}: {error}'
            )

    def save_mask(self, regions):
        self.load_mask()
        self.mask['regions'] = normalize_regions(regions)
        if 'beam_indices' not in self.mask:
            self.mask['beam_indices'] = []
        os.makedirs(os.path.dirname(self.mask_file), exist_ok=True)
        with open(self.mask_file, 'w', encoding='utf-8') as file:
            json.dump(self.mask, file, indent=2, sort_keys=True)
        self.get_logger().info(
            f'Saved {len(self.mask["regions"])} LiDAR ignore regions.'
        )

    def make_handler(self):
        node = self

        class ViewerHandler(BaseHTTPRequestHandler):
            def log_message(self, format, *args):
                return

            def do_GET(self):
                if self.path in ('/', '/index.html'):
                    self.send_response(200)
                    self.send_header('Content-Type', 'text/html; charset=utf-8')
                    self.end_headers()
                    self.wfile.write(VIEWER_HTML.encode('utf-8'))
                    return

                if self.path != '/api/state':
                    self.send_error(404)
                    return

                with node.lock:
                    payload = {
                        'raw_scan': node.latest_raw_scan,
                        'filtered_scan': node.latest_filtered_scan,
                        'mask': node.mask,
                    }
                self.send_json(payload)

            def do_POST(self):
                if self.path != '/api/mask':
                    self.send_error(404)
                    return

                length = int(self.headers.get('Content-Length', '0'))
                try:
                    body = self.rfile.read(length).decode('utf-8')
                    payload = json.loads(body)
                    node.save_mask(payload.get('regions', []))
                except (OSError, json.JSONDecodeError, ValueError) as error:
                    self.send_response(400)
                    self.send_header('Content-Type', 'text/plain')
                    self.end_headers()
                    self.wfile.write(str(error).encode('utf-8'))
                    return

                with node.lock:
                    response = {
                        'raw_scan': node.latest_raw_scan,
                        'filtered_scan': node.latest_filtered_scan,
                        'mask': node.mask,
                    }
                self.send_json(response)

            def send_json(self, payload):
                body = json.dumps(payload).encode('utf-8')
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Cache-Control', 'no-store')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)

        return ViewerHandler

    def destroy_node(self):
        self.server.shutdown()
        self.server.server_close()
        super().destroy_node()


def normalize_mask(payload):
    return {
        'regions': normalize_regions(payload.get('regions', [])),
        'beam_indices': [
            int(index)
            for index in payload.get('beam_indices', [])
            if isinstance(index, int)
        ],
    }


def normalize_regions(regions):
    clean_regions = []
    for region_index, region in enumerate(regions):
        points = []
        for point in region.get('points', []):
            try:
                points.append({
                    'x': float(point['x']),
                    'y': float(point['y']),
                })
            except (KeyError, TypeError, ValueError):
                continue
        if len(points) >= 3:
            clean_regions.append({
                'name': region.get('name') or f'ignore_{region_index + 1}',
                'points': points,
            })
    return clean_regions


def main(args=None):
    rclpy.init(args=args)
    node = LidarWebViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
