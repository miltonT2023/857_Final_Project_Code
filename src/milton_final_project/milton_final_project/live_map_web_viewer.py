import json
import math
import os
from html import escape
from http.server import BaseHTTPRequestHandler
from http.server import ThreadingHTTPServer
from threading import Lock
from threading import Thread

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener

from milton_final_project.filter_saved_map import filter_saved_map
from milton_final_project.map_3d_viewer import load_labels
from milton_final_project.map_3d_viewer import write_labels
from milton_final_project.save_latest_map import write_pgm
from milton_final_project.save_latest_map import write_yaml


DEFAULT_MAP_DIR = '/home/nvidia/857_Final_Project_Code/maps'


VIEWER_HTML = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>__PAGE_TITLE__</title>
  <style>
    body {
      margin: 0;
      background: #111318;
      color: #eef2f6;
      font-family: Arial, Helvetica, sans-serif;
      overflow: hidden;
    }
    header {
      height: 52px;
      display: flex;
      align-items: center;
      gap: 16px;
      padding: 0 16px;
      background: #181d24;
      border-bottom: 1px solid #2b333d;
      box-sizing: border-box;
    }
    h1 {
      margin: 0;
      font-size: 16px;
    }
    #status {
      color: #aeb8c4;
      font-size: 13px;
      overflow: hidden;
      text-overflow: ellipsis;
      white-space: nowrap;
    }
    #diag {
      color: #d5dde7;
      font-size: 12px;
      white-space: nowrap;
    }
    .controls {
      margin-left: auto;
      display: flex;
      align-items: center;
      gap: 8px;
    }
    .map-actions {
      display: __MAP_ACTION_DISPLAY__;
      align-items: center;
      gap: 8px;
    }
    button {
      border: 1px solid #3a4654;
      border-radius: 6px;
      background: #202833;
      color: #eef2f6;
      cursor: pointer;
      font-size: 12px;
      padding: 7px 10px;
      white-space: nowrap;
    }
    button:hover {
      background: #2b3542;
    }
    button:disabled {
      cursor: wait;
      opacity: 0.55;
    }
    #actionStatus {
      color: #9fd8ff;
      font-size: 12px;
      min-width: 170px;
      overflow: hidden;
      text-overflow: ellipsis;
      white-space: nowrap;
    }
    canvas {
      display: block;
      width: 100vw;
      height: calc(100vh - 52px);
      background: #0c0f13;
    }
  </style>
</head>
<body>
  <header>
    <h1>__PAGE_TITLE__</h1>
    <div id="status">Waiting for /map...</div>
    <div class="controls">
      <div class="map-actions">
        <button id="saveMap">Save Map</button>
        <button id="filterMap">Filter Map</button>
        <button id="saveStart">Save Start</button>
        <div id="actionStatus"></div>
      </div>
      <div id="diag">scan: -- | odom: -- | map: --</div>
    </div>
  </header>
  <canvas id="map"></canvas>
  <script>
    const canvas = document.getElementById('map');
    const ctx = canvas.getContext('2d');
    const statusEl = document.getElementById('status');
    const diagEl = document.getElementById('diag');
    const actionStatusEl = document.getElementById('actionStatus');
    const saveMapEl = document.getElementById('saveMap');
    const filterMapEl = document.getElementById('filterMap');
    const saveStartEl = document.getElementById('saveStart');
    let latest = null;

    function resize() {
      const ratio = window.devicePixelRatio || 1;
      canvas.width = Math.floor(window.innerWidth * ratio);
      canvas.height = Math.floor((window.innerHeight - 52) * ratio);
      ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
      draw();
    }

    function colorFor(value) {
      if (value < 0) return [72, 78, 88, 255];
      if (value >= 65) return [18, 23, 30, 255];
      return [232, 238, 244, 255];
    }

    function draw() {
      ctx.clearRect(0, 0, canvas.clientWidth, canvas.clientHeight);
      if (!latest || latest.width <= 0 || latest.height <= 0) {
        ctx.fillStyle = '#aeb8c4';
        ctx.font = '16px Arial';
        ctx.fillText('Waiting for a non-empty /map...', 18, 34);
        return;
      }

      const image = ctx.createImageData(latest.width, latest.height);
      for (let y = 0; y < latest.height; y += 1) {
        for (let x = 0; x < latest.width; x += 1) {
          const sourceIndex = (latest.height - 1 - y) * latest.width + x;
          const targetIndex = (y * latest.width + x) * 4;
          const color = colorFor(latest.data[sourceIndex]);
          image.data[targetIndex] = color[0];
          image.data[targetIndex + 1] = color[1];
          image.data[targetIndex + 2] = color[2];
          image.data[targetIndex + 3] = color[3];
        }
      }

      const offscreen = document.createElement('canvas');
      offscreen.width = latest.width;
      offscreen.height = latest.height;
      offscreen.getContext('2d').putImageData(image, 0, 0);

      const availableWidth = canvas.clientWidth;
      const availableHeight = canvas.clientHeight;
      const scale = Math.min(
        availableWidth / latest.width,
        availableHeight / latest.height
      ) * 0.95;
      const drawWidth = latest.width * scale;
      const drawHeight = latest.height * scale;
      const drawX = (availableWidth - drawWidth) / 2;
      const drawY = (availableHeight - drawHeight) / 2;

      ctx.imageSmoothingEnabled = false;
      ctx.drawImage(offscreen, drawX, drawY, drawWidth, drawHeight);
      drawRobot(drawX, drawY, scale);
    }

    function mapToScreen(x, y, drawX, drawY, scale) {
      const origin = latest.origin || [0, 0, 0];
      const pixelX = (x - origin[0]) / latest.resolution;
      const pixelY = latest.height - ((y - origin[1]) / latest.resolution);
      return [drawX + pixelX * scale, drawY + pixelY * scale];
    }

    function drawRobot(drawX, drawY, scale) {
      if (!latest.robot_pose) {
        return;
      }
      const pose = latest.robot_pose;
      const [x, y] = mapToScreen(pose.x, pose.y, drawX, drawY, scale);
      const radius = Math.max(8, 0.18 / latest.resolution * scale);
      const headingLength = Math.max(22, 0.45 / latest.resolution * scale);
      // Rotate the marker so zero yaw points downward on the screen,
      // matching the robot's physical front orientation for this setup.
      const yaw = -pose.yaw + Math.PI / 2;
      const hx = x + Math.cos(yaw) * headingLength;
      const hy = y + Math.sin(yaw) * headingLength;

      ctx.save();
      ctx.lineWidth = 4;
      ctx.strokeStyle = '#3aa7ff';
      ctx.fillStyle = '#3aa7ff';
      ctx.beginPath();
      ctx.arc(x, y, radius, 0, Math.PI * 2);
      ctx.fill();
      ctx.strokeStyle = '#07131f';
      ctx.lineWidth = 2;
      ctx.stroke();

      ctx.strokeStyle = '#3aa7ff';
      ctx.lineWidth = 5;
      ctx.lineCap = 'round';
      ctx.beginPath();
      ctx.moveTo(x, y);
      ctx.lineTo(hx, hy);
      ctx.stroke();

      ctx.translate(hx, hy);
      ctx.rotate(yaw);
      ctx.beginPath();
      ctx.moveTo(0, 0);
      ctx.lineTo(-12, -7);
      ctx.lineTo(-12, 7);
      ctx.closePath();
      ctx.fill();
      ctx.restore();
    }

    async function pollMap() {
      try {
        const response = await fetch('/api/map', {cache: 'no-store'});
        if (!response.ok) {
          statusEl.textContent = await response.text();
          latest = null;
        } else {
          latest = await response.json();
          statusEl.textContent = latest.status;
          diagEl.textContent = latest.diagnostics;
        }
      } catch (error) {
        statusEl.textContent = `Viewer disconnected: ${error}`;
        diagEl.textContent = 'scan: -- | odom: -- | map: --';
        latest = null;
      }
      draw();
    }

    async function runAction(path, label) {
      const buttons = [saveMapEl, filterMapEl, saveStartEl];
      buttons.forEach((button) => { button.disabled = true; });
      actionStatusEl.textContent = `${label}...`;
      try {
        const response = await fetch(path, {method: 'POST'});
        const payload = await response.json();
        if (!response.ok || !payload.ok) {
          actionStatusEl.textContent = payload.error || `${label} failed`;
          return;
        }
        actionStatusEl.textContent = payload.message || `${label} done`;
      } catch (error) {
        actionStatusEl.textContent = `${label} failed: ${error}`;
      } finally {
        buttons.forEach((button) => { button.disabled = false; });
      }
    }

    window.addEventListener('resize', resize);
    saveMapEl.addEventListener('click', () => runAction('/api/save_map', 'Saving map'));
    filterMapEl.addEventListener('click', () => runAction('/api/filter_map', 'Filtering map'));
    saveStartEl.addEventListener('click', () => runAction('/api/save_start', 'Saving start'));
    resize();
    pollMap();
    setInterval(pollMap, 1000);
  </script>
</body>
</html>
"""


def quaternion_to_yaw(quaternion):
    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


class LiveMapWebViewer(Node):
    def __init__(self):
        super().__init__('live_map_web_viewer')

        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8090)
        self.declare_parameter('map_dir', DEFAULT_MAP_DIR)
        self.declare_parameter('map_name_prefix', 'slam_toolbox_map')
        self.declare_parameter('start_label', 'robot_start')
        self.declare_parameter('start_aliases', 'start,home,original')
        self.declare_parameter('page_title', 'QBot Live SLAM Map')
        self.declare_parameter('show_map_actions', True)

        self.lock = Lock()
        self.latest_map = None
        self.latest_map_msg = None
        self.latest_scan_stamp = None
        self.latest_odom_stamp = None
        self.latest_robot_pose = None
        self.latest_robot_pose_stamp = None
        self.last_saved_yaml = None

        map_topic = self.get_parameter('map_topic').value
        scan_topic = self.get_parameter('scan_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        host = self.get_parameter('host').value
        port = int(self.get_parameter('port').value)
        self.map_dir = os.path.abspath(
            os.path.expanduser(self.get_parameter('map_dir').value)
        )
        self.map_name_prefix = self.get_parameter('map_name_prefix').value
        self.start_label = self.get_parameter('start_label').value
        self.start_aliases = self.get_parameter('start_aliases').value
        self.page_title = self.get_parameter('page_title').value
        self.show_map_actions = bool(
            self.get_parameter('show_map_actions').value
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.create_subscription(OccupancyGrid, map_topic, self.map_callback, map_qos)
        self.create_subscription(LaserScan, scan_topic, self.scan_callback, sensor_qos)
        self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)

        handler = self.make_handler()
        self.server = ThreadingHTTPServer((host, port), handler)
        self.server_thread = Thread(target=self.server.serve_forever, daemon=True)
        self.server_thread.start()

        self.get_logger().info(
            f'Streaming live {map_topic} viewer at http://{host}:{port}'
        )

    def map_callback(self, msg):
        self.update_robot_pose()
        status = (
            f'{msg.info.width} x {msg.info.height} @ '
            f'{msg.info.resolution:.3f} m/pix'
        )
        if msg.info.width == 0 or msg.info.height == 0:
            status = (
                'Map topic is alive, but it is still empty. '
                'Stop duplicate SLAM launches and drive slowly until SLAM '
                'Toolbox receives scan + odom.'
            )

        payload = {
            'width': msg.info.width,
            'height': msg.info.height,
            'resolution': msg.info.resolution,
            'status': status,
            'diagnostics': self.diagnostics_text(),
            'origin': [
                msg.info.origin.position.x,
                msg.info.origin.position.y,
                msg.info.origin.position.z,
            ],
            'robot_pose': self.latest_robot_pose,
            'data': list(msg.data),
        }
        with self.lock:
            self.latest_map = payload
            self.latest_map_msg = msg

    def scan_callback(self, msg):
        with self.lock:
            self.latest_scan_stamp = self.get_clock().now()

    def odom_callback(self, msg):
        with self.lock:
            self.latest_odom_stamp = self.get_clock().now()

    def update_robot_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.robot_frame,
                Time(),
            )
        except TransformException:
            return

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        pose = {
            'x': translation.x,
            'y': translation.y,
            'yaw': quaternion_to_yaw(rotation),
        }
        with self.lock:
            self.latest_robot_pose = pose
            self.latest_robot_pose_stamp = self.get_clock().now()

    def diagnostics_text(self):
        now = self.get_clock().now()

        def age_text(stamp):
            if stamp is None:
                return 'missing'
            age = (now - stamp).nanoseconds / 1_000_000_000.0
            if age > 3.0:
                return f'stale {age:.1f}s'
            return 'ok'

        return (
            f'scan: {age_text(self.latest_scan_stamp)} | '
            f'odom: {age_text(self.latest_odom_stamp)} | '
            f'robot: {age_text(self.latest_robot_pose_stamp)}'
        )

    def saved_map_paths(self):
        base_path = os.path.join(self.map_dir, self.map_name_prefix)
        return f'{base_path}.yaml', f'{base_path}.pgm'

    def save_current_map(self):
        with self.lock:
            map_msg = self.latest_map_msg
        if map_msg is None:
            raise RuntimeError('No live map has been received yet.')

        os.makedirs(self.map_dir, exist_ok=True)
        yaml_path, image_path = self.saved_map_paths()
        write_pgm(image_path, map_msg, clean_map=False)
        write_yaml(yaml_path, os.path.basename(image_path), map_msg)
        self.last_saved_yaml = yaml_path
        return yaml_path

    def filter_current_map(self):
        yaml_path = self.last_saved_yaml or self.saved_map_paths()[0]
        if not os.path.exists(yaml_path):
            raise RuntimeError('Save the map before filtering it.')

        output_yaml, _, _, _ = filter_saved_map(
            yaml_path,
            overwrite=True,
            suffix='_filtered',
            min_component_cells=12,
            close_kernel_cells=5,
            free_open_kernel_cells=5,
            occupied_pixel_threshold=25,
        )
        self.last_saved_yaml = output_yaml
        return output_yaml

    def save_start_pose(self):
        self.update_robot_pose()
        with self.lock:
            pose = dict(self.latest_robot_pose) if self.latest_robot_pose else None
        if pose is None:
            raise RuntimeError('No robot pose is available yet.')

        yaml_path = self.last_saved_yaml or self.saved_map_paths()[0]
        if not os.path.exists(yaml_path):
            raise RuntimeError('Save the map before saving robot start.')

        map_name = os.path.basename(yaml_path)
        # Reset old labels so the saved start pose becomes the canonical
        # origin set for this map.
        labels = {}
        label_names = [self.start_label]
        label_names.extend(
            alias.strip()
            for alias in self.start_aliases.split(',')
            if alias.strip()
        )
        clean_label_names = []
        for label_name in label_names:
            if label_name not in clean_label_names:
                clean_label_names.append(label_name)
        for label_name in clean_label_names:
            labels[label_name] = dict(pose)
        write_labels(self.map_dir, map_name, labels)
        return map_name, clean_label_names

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
                    body = VIEWER_HTML.replace(
                        '__PAGE_TITLE__',
                        escape(str(node.page_title)),
                    ).replace(
                        '__MAP_ACTION_DISPLAY__',
                        'flex' if node.show_map_actions else 'none',
                    )
                    self.wfile.write(body.encode('utf-8'))
                    return

                if self.path != '/api/map':
                    self.send_error(404)
                    return

                node.update_robot_pose()
                with node.lock:
                    payload = dict(node.latest_map) if node.latest_map else None
                    if payload is not None:
                        payload['robot_pose'] = node.latest_robot_pose
                        payload['diagnostics'] = node.diagnostics_text()

                if payload is None:
                    self.send_response(503)
                    self.send_header('Content-Type', 'text/plain')
                    self.end_headers()
                    self.wfile.write(
                        (
                            'No /map message received yet. '
                            + node.diagnostics_text()
                        ).encode('utf-8')
                    )
                    return

                body = json.dumps(payload).encode('utf-8')
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Cache-Control', 'no-store')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_POST(self):
                if self.path == '/api/save_map':
                    self.run_action(node.save_current_map, 'Saved map')
                    return
                if self.path == '/api/filter_map':
                    self.run_action(node.filter_current_map, 'Filtered map')
                    return
                if self.path == '/api/save_start':
                    self.run_action(node.save_start_pose, 'Saved start pose')
                    return

                self.send_error(404)

            def run_action(self, action, success_prefix):
                try:
                    result = action()
                    if isinstance(result, tuple):
                        detail = ', '.join(str(item) for item in result)
                    else:
                        detail = str(result)
                    payload = {
                        'ok': True,
                        'message': f'{success_prefix}: {detail}',
                    }
                except Exception as exc:
                    payload = {
                        'ok': False,
                        'error': str(exc),
                    }
                    body = json.dumps(payload).encode('utf-8')
                    self.send_response(500)
                    self.send_header('Content-Type', 'application/json')
                    self.send_header('Cache-Control', 'no-store')
                    self.send_header('Content-Length', str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return

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


def main(args=None):
    rclpy.init(args=args)
    node = LiveMapWebViewer()

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
