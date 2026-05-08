from dataclasses import dataclass
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import ctypes
import json
import math
import os
from pathlib import Path
import queue
from concurrent.futures import ThreadPoolExecutor
import threading
import textwrap
import time


def preload_static_tls_libraries():
    libgomp_path = '/lib/aarch64-linux-gnu/libgomp.so.1'
    if not Path(libgomp_path).exists():
        return

    try:
        ctypes.CDLL(libgomp_path, mode=ctypes.RTLD_GLOBAL)
    except OSError as exc:
        print(f'Warning: failed to preload {libgomp_path}: {exc}', flush=True)


preload_static_tls_libraries()

from ament_index_python.packages import get_package_share_directory
import cv2
from cv_bridge import CvBridge
import numpy as np
import pygame
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .robot_interpreter import RobotInterpreter
from .seic_directory import SeicDirectory


FACE_STREAM_HTML = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Milton Face Display</title>
  <style>
    :root {
      color-scheme: dark;
      font-family: Inter, ui-sans-serif, system-ui, -apple-system,
        BlinkMacSystemFont, "Segoe UI", sans-serif;
      background: #0a0f14;
      color: #eef5f2;
    }
    * {
      box-sizing: border-box;
    }
    body {
      margin: 0;
      min-height: 100vh;
      background: #0a0f14;
      display: grid;
      place-items: center;
      padding: 20px;
    }
    main {
      width: min(1120px, 100%);
    }
    header {
      display: flex;
      align-items: end;
      justify-content: space-between;
      gap: 16px;
      margin-bottom: 14px;
    }
    h1 {
      margin: 0;
      font-size: 28px;
      line-height: 1.05;
    }
    p {
      margin: 7px 0 0;
      color: #9fb0ac;
      font-size: 14px;
    }
    .pill {
      display: inline-flex;
      align-items: center;
      gap: 8px;
      min-height: 34px;
      padding: 0 12px;
      border: 1px solid #24323a;
      border-radius: 999px;
      background: #111a21;
      color: #d1dfdc;
      font-size: 14px;
      white-space: nowrap;
    }
    .dot {
      width: 9px;
      height: 9px;
      border-radius: 50%;
      background: #66ffb2;
      box-shadow: 0 0 16px rgba(102, 255, 178, 0.75);
    }
    .pill.listening .dot {
      background: #ff4fd8;
      box-shadow: 0 0 18px rgba(255, 79, 216, 0.85);
      animation: pulse 0.9s ease-in-out infinite alternate;
    }
    @keyframes pulse {
      from {
        transform: scale(0.75);
        opacity: 0.65;
      }
      to {
        transform: scale(1.25);
        opacity: 1;
      }
    }
    .frame {
      overflow: hidden;
      border: 1px solid #22313a;
      border-radius: 8px;
      background: #05080b;
      box-shadow: 0 18px 70px rgba(0, 0, 0, 0.38);
    }
    img {
      display: block;
      width: 100%;
      height: auto;
      aspect-ratio: 1024 / 600;
      object-fit: contain;
    }
    form {
      display: grid;
      grid-template-columns: minmax(0, 1fr) auto;
      gap: 10px;
      margin-top: 14px;
    }
    input {
      min-width: 0;
      height: 44px;
      border: 1px solid #273942;
      border-radius: 8px;
      background: #111a21;
      color: #eef5f2;
      font: inherit;
      padding: 0 13px;
      outline: none;
    }
    input:focus {
      border-color: #66d6ff;
      box-shadow: 0 0 0 3px rgba(102, 214, 255, 0.16);
    }
    button {
      min-height: 44px;
      border: 1px solid #2d4853;
      border-radius: 8px;
      background: #dff7ef;
      color: #102019;
      font: inherit;
      font-weight: 750;
      padding: 0 16px;
      cursor: pointer;
    }
    .actions {
      display: flex;
      flex-wrap: wrap;
      gap: 10px;
      margin-top: 10px;
    }
    .actions button {
      background: #111a21;
      color: #d7e7e3;
    }
    .status {
      min-height: 20px;
      margin-top: 10px;
      color: #9fb0ac;
      font-size: 13px;
    }
    @media (max-width: 720px) {
      body {
        padding: 12px;
      }
      header {
        align-items: start;
        flex-direction: column;
      }
      form {
        grid-template-columns: 1fr;
      }
      h1 {
        font-size: 23px;
      }
    }
  </style>
</head>
<body>
  <main>
    <header>
      <div>
        <h1>Milton Face Display</h1>
        <p>Live stream of the robot face screen.</p>
      </div>
      <div id="micPill" class="pill"><span class="dot"></span><span id="micLabel">Live</span></div>
    </header>
    <section class="frame" aria-label="Robot face display stream">
      <img src="/stream.mjpg" alt="Live robot face display">
    </section>
    <form id="commandForm">
      <input id="commandText" maxlength="40" autocomplete="off"
        placeholder="Person or room to find">
      <button type="submit">Send</button>
    </form>
    <div class="actions">
      <button type="button" data-submit="yes">Yes</button>
      <button type="button" data-submit="no">No</button>
      <button type="button" id="clearButton">Clear</button>
    </div>
    <div id="status" class="status">Connected to face display.</div>
  </main>
  <script>
    const form = document.getElementById('commandForm');
    const input = document.getElementById('commandText');
    const status = document.getElementById('status');
    const clearButton = document.getElementById('clearButton');
    const micPill = document.getElementById('micPill');
    const micLabel = document.getElementById('micLabel');

    async function sendCommand(action, text = null) {
      const body = text === null ? { action } : { action, text };
      const response = await fetch('/command', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(body),
      });
      if (!response.ok) {
        throw new Error(`Command failed: ${response.status}`);
      }
      return response.json();
    }

    form.addEventListener('submit', async (event) => {
      event.preventDefault();
      const text = input.value.trim();
      if (!text) return;
      try {
        await sendCommand('submit', text);
        input.value = '';
        status.textContent = `Sent: ${text}`;
      } catch (error) {
        status.textContent = error.message;
      }
    });

    document.querySelectorAll('[data-submit]').forEach((button) => {
      button.addEventListener('click', async () => {
        const text = button.dataset.submit;
        try {
          await sendCommand('submit', text);
          status.textContent = `Sent: ${text}`;
        } catch (error) {
          status.textContent = error.message;
        }
      });
    });

    clearButton.addEventListener('click', async () => {
      try {
        await sendCommand('clear');
        input.value = '';
        status.textContent = 'Cleared input.';
      } catch (error) {
        status.textContent = error.message;
      }
    });

    document.addEventListener('keydown', async (event) => {
      if (event.code !== 'Space') return;
      event.preventDefault();
      try {
        await sendCommand('start_stt');
        status.textContent = 'Listening...';
      } catch (error) {
        status.textContent = error.message;
      }
    });

    async function refreshStatus() {
      try {
        const response = await fetch('/status.json', { cache: 'no-store' });
        if (!response.ok) throw new Error(`Status failed: ${response.status}`);
        const state = await response.json();
        if (document.activeElement !== input) {
          input.value = state.input || '';
        }
        micPill.classList.toggle('listening', Boolean(state.listening));
        micLabel.textContent = state.listening ? 'Mic listening' : 'Live';
        if (
          state.stt_last_text &&
          state.stt_last_text !== input.value &&
          document.activeElement !== input
        ) {
          input.value = state.stt_last_text;
        }
        if (state.listening) {
          status.textContent = 'Listening... speak now.';
        } else if (state.stt_status) {
          status.textContent = state.stt_status;
        }
      } catch (error) {
        status.textContent = error.message;
      }
    }

    setInterval(refreshStatus, 500);
    refreshStatus();
  </script>
</body>
</html>
"""


@dataclass(frozen=True)
class AssetExpression:
    name: str
    label: str
    folder: str
    eye_height: int
    eye_gap: int
    baseline_offset: int


class FaceDisplayNode(Node):
    def __init__(self):
        super().__init__('face_display_node')

        self.declare_parameter('width', 1024)
        self.declare_parameter('height', 600)
        self.declare_parameter('fullscreen', True)
        self.declare_parameter('show_help', False)
        self.declare_parameter('preview_topic', '/yolo/annotated_image')
        self.declare_parameter('person_target_topic', '/yolo/person_target')
        self.declare_parameter('initial_expression', 'neutral')
        self.declare_parameter(
            'waiting_message',
            'I am the SEIC navigation robot. Please enter the person or '
            'room you are trying to find.',
        )
        self.declare_parameter('response_duration_sec', 10.0)
        self.declare_parameter('confirmation_timeout_sec', 60.0)
        self.declare_parameter('navigation_timeout_sec', 0.0)
        default_phrase_log = (
            Path.home()
            / 'Milton_Final_Project'
            / 'runtime_logs'
            / 'face_gui_phrases.txt'
        )
        self.declare_parameter('phrase_log_file', str(default_phrase_log))
        self.declare_parameter('speak_phrases', True)
        self.declare_parameter('speech_rate', 125)
        self.declare_parameter('speech_volume', 0.65)
        self.declare_parameter('speech_voice_id', 'gmw/en-us')
        self.declare_parameter('speech_chars_per_sec', 13.0)
        self.declare_parameter('stt_enabled', True)
        self.declare_parameter('stt_silence_timeout_sec', 2.0)
        self.declare_parameter('stt_backend', 'faster_whisper')
        self.declare_parameter('stt_model_size', 'base')
        self.declare_parameter('stt_model_path', '')
        self.declare_parameter('stt_device', 'auto')
        self.declare_parameter('stt_compute_type', 'auto')
        self.declare_parameter('stt_local_files_only', False)
        self.declare_parameter('frame_rate', 20.0)
        self.declare_parameter('web_stream_enabled', True)
        self.declare_parameter('web_stream_host', '0.0.0.0')
        self.declare_parameter('web_stream_port', 8080)
        self.declare_parameter('web_stream_jpeg_quality', 85)
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.show_help = bool(self.get_parameter('show_help').value)
        self.response_duration_sec = float(
            self.get_parameter('response_duration_sec').value
        )
        self.confirmation_timeout_sec = float(
            self.get_parameter('confirmation_timeout_sec').value
        )
        self.navigation_timeout_sec = float(
            self.get_parameter('navigation_timeout_sec').value
        )
        self.preview_topic = self.get_parameter('preview_topic').value
        self.person_target_topic = self.get_parameter('person_target_topic').value
        self.phrase_log_file = Path(
            self.get_parameter('phrase_log_file').value
        ).expanduser()
        self.phrase_log_file.parent.mkdir(parents=True, exist_ok=True)
        self.speak_phrases = bool(self.get_parameter('speak_phrases').value)
        self.speech_rate = int(self.get_parameter('speech_rate').value)
        self.speech_volume = float(self.get_parameter('speech_volume').value)
        self.speech_voice_id = self.get_parameter('speech_voice_id').value
        self.speech_chars_per_sec = float(
            self.get_parameter('speech_chars_per_sec').value
        )
        self.stt_enabled = bool(self.get_parameter('stt_enabled').value)
        self.stt_silence_timeout_sec = float(
            self.get_parameter('stt_silence_timeout_sec').value
        )
        self.stt_backend = str(self.get_parameter('stt_backend').value)
        self.stt_model_size = self.get_parameter('stt_model_size').value
        stt_model_path = str(
            self.get_parameter('stt_model_path').value
        ).strip()
        self.stt_model_path = (
            Path(stt_model_path).expanduser() if stt_model_path else None
        )
        self.stt_device = str(self.get_parameter('stt_device').value)
        self.stt_compute_type = str(
            self.get_parameter('stt_compute_type').value
        )
        self.stt_local_files_only = bool(
            self.get_parameter('stt_local_files_only').value
        )
        self.frame_rate = float(self.get_parameter('frame_rate').value)
        self.web_stream_enabled = bool(
            self.get_parameter('web_stream_enabled').value
        )
        self.web_stream_host = self.get_parameter('web_stream_host').value
        self.web_stream_port = int(self.get_parameter('web_stream_port').value)
        self.web_stream_jpeg_quality = int(
            self.get_parameter('web_stream_jpeg_quality').value
        )
        self.expression_index = 0

        pygame.init()
        pygame.font.init()

        display_flags = pygame.FULLSCREEN if self.get_parameter('fullscreen').value else 0
        self.screen = pygame.display.set_mode((self.width, self.height), display_flags)
        self.clock = pygame.time.Clock()

        self.bg = (236, 244, 248)
        self.label = (79, 105, 120)
        self.help_color = (106, 130, 144)
        self.message_bg = (250, 253, 255, 238)
        self.message_outline = (132, 198, 214)
        self.message_text = (48, 68, 82)
        self.navigation_bg = (104, 193, 143)
        self.navigation_bg_dark = (66, 144, 104)
        self.navigation_text = (244, 255, 247)
        self.detected_border_color = (52, 199, 89)
        self.idle_border_color = (44, 132, 255)
        self.border_thickness = 22

        self.cx = self.width // 2
        self.bottom_panel_height = 210
        self.cy = (self.height - self.bottom_panel_height) // 2 - 40
        self.face_size = min(self.width, self.height - self.bottom_panel_height) - 40

        share_dir = Path(get_package_share_directory('milton_final_project'))
        self.assets_dir = share_dir / 'assets' / 'kaia_face'
        self.background = self.load_image('happy/background-min.png')

        self.expression_order = [
            'neutral',
            'happy',
            'confused',
        ]
        self.expressions = {
            'neutral': AssetExpression('neutral', 'Neutral', 'happy', 58, 108, 6),
            'happy': AssetExpression('happy', 'Happy', 'happy', 58, 108, 6),
            'confused': AssetExpression('confused', 'Confused', 'happy', 58, 108, 6),
        }
        self.loaded_eyes = {
            name: {
                'left': self.load_image(f'{expr.folder}/left-eye.png'),
                'right': self.load_image(f'{expr.folder}/right-eye.png'),
            }
            for name, expr in self.expressions.items()
        }

        self.waiting_message = self.get_parameter('waiting_message').value
        self.current_message = self.waiting_message
        self.idle_expression = self.normalize_expression(
            self.get_parameter('initial_expression').value
        )
        self.active_expression = self.idle_expression
        self.override_expression = None
        self.override_message = None
        self.override_until = None
        self.input_buffer = ''
        self.pending_future = None
        self.pending_destination_label = None
        self.awaiting_confirmation = False
        self.navigation_mode_active = False
        self.state_timeout_at = None
        self.assistant_pool = ThreadPoolExecutor(max_workers=1)
        self.speech_queue = queue.Queue()
        self.speech_stop = threading.Event()
        self.speech_thread = None
        self.speech_display_text = ''
        self.speech_display_started_at = None
        self.speech_display_until = None
        self.last_spoken_message = None
        self.stt_model = None
        self.stt_model_kind = None
        self.listening = False
        self.listening_until = None
        self.listening_thread = None
        self.stt_timeout_sec = 15.0
        self.stt_sample_rate = 16000
        self.stt_status = 'Press Spacebar to speak.'
        self.stt_last_text = ''
        self.stt_result_queue = queue.Queue()
        self.interpreter = RobotInterpreter()
        self.directory = SeicDirectory()
        self.bridge = CvBridge()
        self.preview_surface = None
        self.preview_size = (320, 180)
        self.person_detected = False
        self.face_glow_color = (196, 168, 255, 92)
        self.face_halo_color = (156, 126, 235, 78)
        self.stream_lock = threading.Lock()
        self.latest_stream_jpeg = None
        self.stream_frame_count = 0
        self.stream_server = None
        self.stream_thread = None
        self.web_command_queue = queue.Queue()

        self.set_expression(self.active_expression)

        self.font = pygame.font.SysFont('arial', 28, bold=True)
        self.small_font = pygame.font.SysFont('arial', 18)
        self.message_font = pygame.font.SysFont('arial', 30, bold=True)
        self.navigation_font = pygame.font.SysFont('arial', 58, bold=True)
        self.input_font = pygame.font.SysFont('arial', 26)
        self.help_lines = []

        self.expression_sub = self.create_subscription(
            String,
            '/face/expression',
            self.expression_callback,
            10,
        )
        self.message_sub = self.create_subscription(
            String,
            '/face/message',
            self.message_callback,
            10,
        )
        self.preview_sub = self.create_subscription(
            Image,
            self.preview_topic,
            self.preview_callback,
            10,
        )
        self.person_target_sub = self.create_subscription(
            String,
            self.person_target_topic,
            self.person_target_callback,
            10,
        )
        self.light_state_pub = self.create_publisher(String, '/robot/light_state', 10)
        self.label_pub = self.create_publisher(String, '/label', 10)
        self.last_light_state = None

        if self.speak_phrases:
            self.speech_thread = threading.Thread(
                target=self.speech_loop,
                daemon=True,
            )
            self.speech_thread.start()
            self.speak_robot_message(self.current_message, save=True)

        if self.web_stream_enabled:
            self.start_web_stream()

        self.timer = self.create_timer(1.0 / max(5.0, self.frame_rate), self.update_frame)
        self.publish_light_state()
        self.get_logger().info(f'Face monitor ready with assets from: {self.assets_dir}')
        self.get_logger().info(f'Face preview subscribed to: {self.preview_topic}')
        self.get_logger().info(
            f'Face border subscribed to person target topic: {self.person_target_topic}'
        )
        self.get_logger().info(f'Face GUI phrases will be saved to: {self.phrase_log_file}')
        if self.speak_phrases:
            self.get_logger().info('Face GUI text-to-speech is enabled.')
        if self.stt_enabled:
            self.get_logger().info(
                'Face GUI speech-to-text is enabled '
                '(press Spacebar to listen).'
            )
            self.load_stt_model()

    def start_web_stream(self):
        handler = self.make_stream_handler()
        self.stream_server = ThreadingHTTPServer(
            (self.web_stream_host, self.web_stream_port),
            handler,
        )
        self.stream_thread = threading.Thread(
            target=self.stream_server.serve_forever,
            daemon=True,
        )
        self.stream_thread.start()
        self.get_logger().info(
            'Face display stream ready at '
            f'http://{self.web_stream_host}:{self.web_stream_port}'
        )

    def make_stream_handler(self):
        node = self

        class FaceStreamHandler(BaseHTTPRequestHandler):
            def log_message(self, format, *args):
                return

            def do_GET(self):
                if self.path in ('/', '/index.html'):
                    body = FACE_STREAM_HTML.encode('utf-8')
                    self.send_response(200)
                    self.send_header('Content-Type', 'text/html; charset=utf-8')
                    self.send_header('Content-Length', str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return

                if self.path == '/status.json':
                    body = json.dumps(node.build_web_status()).encode('utf-8')
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json')
                    self.send_header('Cache-Control', 'no-store')
                    self.send_header('Content-Length', str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return

                if self.path != '/stream.mjpg':
                    self.send_error(404)
                    return

                self.send_response(200)
                self.send_header('Age', '0')
                self.send_header('Cache-Control', 'no-cache, private')
                self.send_header('Pragma', 'no-cache')
                self.send_header(
                    'Content-Type',
                    'multipart/x-mixed-replace; boundary=frame',
                )
                self.end_headers()

                while rclpy.ok():
                    with node.stream_lock:
                        frame = node.latest_stream_jpeg

                    if frame is None:
                        time.sleep(0.03)
                        continue

                    try:
                        self.wfile.write(b'--frame\r\n')
                        self.wfile.write(b'Content-Type: image/jpeg\r\n\r\n')
                        self.wfile.write(frame)
                        self.wfile.write(b'\r\n')
                    except (BrokenPipeError, ConnectionResetError):
                        break

            def do_POST(self):
                if self.path != '/command':
                    self.send_error(404)
                    return

                content_length = int(self.headers.get('Content-Length', '0'))
                body = self.rfile.read(content_length)
                try:
                    command = json.loads(body.decode('utf-8') or '{}')
                except json.JSONDecodeError:
                    self.send_error(400, 'Invalid JSON')
                    return

                node.web_command_queue.put(command)
                response = json.dumps({'ok': True}).encode('utf-8')
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(response)))
                self.end_headers()
                self.wfile.write(response)

        return FaceStreamHandler

    def build_web_status(self):
        return {
            'input': self.input_buffer,
            'message': self.current_message,
            'awaiting_confirmation': self.awaiting_confirmation,
            'navigation_mode_active': self.navigation_mode_active,
            'listening': self.listening,
            'stt_enabled': self.stt_enabled,
            'stt_status': self.stt_status,
            'stt_last_text': self.stt_last_text,
            'stream_frame_count': self.stream_frame_count,
        }

    def process_web_commands(self):
        while True:
            try:
                command = self.web_command_queue.get_nowait()
            except queue.Empty:
                return

            action = str(command.get('action', '')).strip()
            text = str(command.get('text', ''))[:40]

            if action == 'submit':
                self.input_buffer = text
                self.process_destination()
            elif action == 'set_text':
                self.input_buffer = text
            elif action == 'backspace':
                self.input_buffer = self.input_buffer[:-1]
            elif action == 'clear':
                self.input_buffer = ''
            elif action == 'reset':
                self.input_buffer = ''
                self.clear_override()
            elif action == 'start_stt':
                self.start_stt_listening()

    def update_stream_frame(self):
        if not self.web_stream_enabled:
            return

        try:
            frame = pygame.surfarray.array3d(self.screen)
            frame = np.transpose(frame, (1, 0, 2))
            frame = np.ascontiguousarray(frame[:, :, ::-1])
            ok, jpeg = cv2.imencode(
                '.jpg',
                frame,
                [cv2.IMWRITE_JPEG_QUALITY, self.web_stream_jpeg_quality],
            )
        except Exception as exc:
            self.get_logger().warning(
                f'Face stream frame capture failed: {exc!r}',
                throttle_duration_sec=2.0,
            )
            return

        if not ok:
            return

        with self.stream_lock:
            self.latest_stream_jpeg = jpeg.tobytes()
            self.stream_frame_count += 1

    def load_image(self, relative_path: str) -> pygame.Surface:
        path = self.assets_dir / relative_path
        if not path.exists():
            raise FileNotFoundError(f'Missing asset: {path}')
        return pygame.image.load(path).convert_alpha()

    def set_expression(self, name: str):
        normalized = self.normalize_expression(name)
        if normalized in self.expressions:
            self.expression_index = self.expression_order.index(normalized)

    def cycle_expression(self, direction: int):
        self.expression_index = (self.expression_index + direction) % len(self.expression_order)
        self.idle_expression = self.expression_order[self.expression_index]
        self.active_expression = self.idle_expression

    def normalize_expression(self, name: str) -> str:
        if name == 'happy':
            return 'happy'
        if name == 'confused':
            return 'confused'
        return 'neutral'

    def current_expression(self) -> AssetExpression:
        return self.expressions[self.expression_order[self.expression_index]]

    def scaled_surface(self, surface: pygame.Surface, target_height: int) -> pygame.Surface:
        width, height = surface.get_size()
        scale = target_height / height
        target_width = max(1, int(width * scale))
        return pygame.transform.smoothscale(surface, (target_width, target_height))

    def wrap_message(self, message: str):
        max_chars = max(18, min(42, int((self.width - 220) / 18)))
        return textwrap.wrap(message, width=max_chars) or ['']

    def fit_input_text(self, message: str, max_width: int) -> str:
        if self.input_font.size(message)[0] <= max_width:
            return message

        shortened = message
        while shortened and self.input_font.size('...' + shortened)[0] > max_width:
            shortened = shortened[1:]
        return '...' + shortened if shortened else ''

    def set_override(self, expression=None, message=None):
        if expression is not None:
            normalized_expression = self.normalize_expression(expression)
            self.override_expression = normalized_expression
            self.active_expression = normalized_expression
            self.set_expression(normalized_expression)

        if message is not None:
            self.override_message = message
            self.show_robot_message(message)

        self.override_until = (
            self.get_clock().now().nanoseconds / 1e9 + self.response_duration_sec
        )

    def clear_override(self):
        self.override_expression = None
        self.override_message = None
        self.override_until = None
        self.pending_destination_label = None
        self.awaiting_confirmation = False
        self.navigation_mode_active = False
        self.state_timeout_at = None
        self.active_expression = self.idle_expression
        self.current_message = self.waiting_message
        self.set_expression(self.idle_expression)
        self.publish_light_state()

    def current_light_state(self) -> str:
        if self.navigation_mode_active:
            return 'navigation'
        if self.awaiting_confirmation:
            return 'confirmation'
        return 'waiting'

    def publish_light_state(self):
        state = self.current_light_state()
        if state == self.last_light_state:
            return

        msg = String()
        msg.data = state
        self.light_state_pub.publish(msg)
        self.last_light_state = state

    def expression_callback(self, msg: String):
        expression = msg.data.strip()
        if expression:
            self.set_override(expression=expression)

    def message_callback(self, msg: String):
        message = msg.data.strip()
        if message:
            self.set_override(message=message)

    def preview_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            rgb_frame = frame[:, :, ::-1]
            rgb_frame = np.ascontiguousarray(rgb_frame)
            self.preview_surface = pygame.image.fromstring(
                rgb_frame.tobytes(),
                (rgb_frame.shape[1], rgb_frame.shape[0]),
                'RGB',
            )
        except Exception as exc:
            self.get_logger().warning(
                f'Preview frame conversion failed: {exc!r}',
                throttle_duration_sec=2.0,
            )

    def person_target_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warning(
                f'Person target decode failed: {exc!r}',
                throttle_duration_sec=2.0,
            )
            self.person_detected = False
            return

        self.person_detected = bool(payload.get('seen', False))

    def current_gaze_offset(self):
        if self.override_expression == 'happy':
            elapsed = self.get_clock().now().nanoseconds / 1e9
            return int(math.sin(elapsed * 3.2) * 8)

        if self.override_until is not None:
            return 0

        elapsed = self.get_clock().now().nanoseconds / 1e9
        return int(math.sin(elapsed * 0.9) * 18)

    def is_blinking(self):
        return False

    def current_face_offset_y(self):
        if self.override_expression == 'happy':
            elapsed = self.get_clock().now().nanoseconds / 1e9
            return int(math.sin(elapsed * 4.0) * 6)
        return 0

    def draw_background(self):
        face_offset_y = self.current_face_offset_y()
        halo = pygame.Surface((self.face_size + 160, self.face_size + 160), pygame.SRCALPHA)
        pygame.draw.ellipse(halo, self.face_halo_color, halo.get_rect())
        halo_rect = halo.get_rect(center=(self.cx, self.cy - 10 + face_offset_y))
        self.screen.blit(halo, halo_rect)

        glow = pygame.Surface((self.face_size + 70, self.face_size + 70), pygame.SRCALPHA)
        pygame.draw.ellipse(glow, self.face_glow_color, glow.get_rect())
        glow_rect = glow.get_rect(center=(self.cx, self.cy - 6 + face_offset_y))
        self.screen.blit(glow, glow_rect)

        shadow = pygame.Surface((self.face_size + 44, self.face_size + 44), pygame.SRCALPHA)
        pygame.draw.ellipse(shadow, (56, 91, 104, 42), shadow.get_rect())
        shadow_rect = shadow.get_rect(center=(self.cx, self.cy + 12 + face_offset_y))
        self.screen.blit(shadow, shadow_rect)

        face = pygame.transform.smoothscale(self.background, (self.face_size, self.face_size))
        face_rect = face.get_rect(center=(self.cx, self.cy + face_offset_y))
        self.screen.blit(face, face_rect)

    def draw_bottom_panel(self):
        panel_surface = pygame.Surface((self.width, self.bottom_panel_height), pygame.SRCALPHA)
        pygame.draw.rect(
            panel_surface,
            (222, 238, 244, 228),
            panel_surface.get_rect(),
            border_radius=28,
        )
        pygame.draw.line(
            panel_surface,
            (159, 207, 217, 255),
            (42, 0),
            (self.width - 42, 0),
            2,
        )
        panel_rect = panel_surface.get_rect(midbottom=(self.cx, self.height))
        self.screen.blit(panel_surface, panel_rect)

    def draw_expression(self):
        expr = self.current_expression()
        eyes = self.loaded_eyes[expr.name]
        left_eye = self.scaled_surface(eyes['left'], expr.eye_height)
        right_eye = self.scaled_surface(eyes['right'], expr.eye_height)

        baseline = self.cy + expr.baseline_offset + self.current_face_offset_y()
        gaze_offset = self.current_gaze_offset()
        left_rect = left_eye.get_rect(
            midbottom=(self.cx - expr.eye_gap + gaze_offset, baseline)
        )
        right_rect = right_eye.get_rect(
            midbottom=(self.cx + expr.eye_gap + gaze_offset, baseline)
        )

        self.screen.blit(left_eye, left_rect)
        self.screen.blit(right_eye, right_rect)

    def draw_message(self):
        lines = self.wrap_message(self.current_message)
        text_surfaces = [
            self.message_font.render(line, True, self.message_text) for line in lines
        ]
        max_width = max(surface.get_width() for surface in text_surfaces)
        line_gap = 8
        total_height = sum(surface.get_height() for surface in text_surfaces)
        total_height += line_gap * max(0, len(text_surfaces) - 1)

        box_width = max_width + 48
        box_height = total_height + 36
        box_surface = pygame.Surface((box_width, box_height), pygame.SRCALPHA)
        pygame.draw.rect(
            box_surface,
            self.message_bg,
            box_surface.get_rect(),
            border_radius=24,
        )
        pygame.draw.rect(
            box_surface,
            self.message_outline,
            box_surface.get_rect(),
            width=2,
            border_radius=24,
        )

        input_box_top = self.height - 64
        message_bottom = input_box_top - 20
        box_rect = box_surface.get_rect(midbottom=(self.cx, message_bottom))
        self.screen.blit(box_surface, box_rect)

        y = box_rect.top + 18
        for surface in text_surfaces:
            text_rect = surface.get_rect(centerx=self.cx, y=y)
            self.screen.blit(surface, text_rect)
            y += surface.get_height() + line_gap

    def draw_input_box(self):
        box_width = min(self.width - 120, 760)
        box_height = 58
        box_surface = pygame.Surface((box_width, box_height), pygame.SRCALPHA)
        pygame.draw.rect(
            box_surface,
            (248, 251, 253, 240),
            box_surface.get_rect(),
            border_radius=18,
        )
        pygame.draw.rect(
            box_surface,
            (132, 198, 214),
            box_surface.get_rect(),
            width=2,
            border_radius=18,
        )

        box_rect = box_surface.get_rect(midbottom=(self.cx, self.height - 8))
        self.screen.blit(box_surface, box_rect)

        if self.listening:
            placeholder = 'Listening...'
        elif self.awaiting_confirmation:
            placeholder = 'Type yes or no'
        elif self.navigation_mode_active:
            placeholder = 'Press Enter to ask for another person or room'
        else:
            placeholder = 'Person or room to find'

        display_text = self.input_buffer if self.input_buffer else placeholder
        text_color = self.message_text if self.input_buffer else (136, 154, 165)
        max_text_width = box_width - 36
        display_text = self.fit_input_text(display_text, max_text_width)
        text_surface = self.input_font.render(display_text, True, text_color)
        text_rect = text_surface.get_rect(midleft=(box_rect.left + 18, box_rect.centery))
        self.screen.blit(text_surface, text_rect)

    def draw_status(self):
        if self.show_help and self.help_lines:
            y = self.height - 62
            for line in self.help_lines:
                text = self.small_font.render(line, True, self.help_color)
                self.screen.blit(text, (26, y))
                y += 22

    def draw_preview(self):
        if self.preview_surface is None:
            return

        preview_width, preview_height = self.preview_size
        frame_surface = pygame.Surface((preview_width + 16, preview_height + 16), pygame.SRCALPHA)
        pygame.draw.rect(
            frame_surface,
            (246, 250, 252, 228),
            frame_surface.get_rect(),
            border_radius=18,
        )
        pygame.draw.rect(
            frame_surface,
            (132, 198, 214, 255),
            frame_surface.get_rect(),
            width=2,
            border_radius=18,
        )

        frame_rect = frame_surface.get_rect(topright=(self.width - 22, 22))
        self.screen.blit(frame_surface, frame_rect)

        scaled_preview = pygame.transform.smoothscale(
            self.preview_surface,
            (preview_width, preview_height),
        )
        preview_rect = scaled_preview.get_rect(center=frame_rect.center)
        self.screen.blit(scaled_preview, preview_rect)

        label_surface = self.small_font.render('Camera Preview', True, self.label)
        label_rect = label_surface.get_rect(
            topright=(frame_rect.right - 10, frame_rect.bottom + 6),
        )
        self.screen.blit(label_surface, label_rect)

    def draw_detection_border(self):
        border_color = (
            self.detected_border_color if self.person_detected else self.idle_border_color
        )
        pygame.draw.rect(
            self.screen,
            border_color,
            self.screen.get_rect(),
            width=self.border_thickness,
            border_radius=18,
        )

    def draw_listening_indicator(self):
        if not self.listening:
            return
        elapsed = self.get_clock().now().nanoseconds / 1e9
        phase = math.sin(elapsed * 4.0)
        alpha = int(140 + phase * 115)
        indicator_color = (102, 255, 178, alpha)
        r = 22
        indicator_surface = pygame.Surface((r * 2 + 24, r * 2 + 24), pygame.SRCALPHA)
        pygame.draw.circle(indicator_surface, indicator_color, (r + 12, r + 12), r)
        rect = indicator_surface.get_rect(midright=(self.width - 30, self.height - 60))
        self.screen.blit(indicator_surface, rect)

    def draw(self):
        if self.navigation_mode_active:
            self.draw_navigation_mode()
            self.draw_detection_border()
            self.draw_listening_indicator()
            pygame.display.flip()
            self.update_stream_frame()
            return

        self.screen.fill(self.bg)
        self.draw_background()
        self.draw_expression()
        self.draw_preview()
        self.draw_bottom_panel()
        self.draw_message()
        self.draw_input_box()
        self.draw_status()
        self.draw_detection_border()
        self.draw_listening_indicator()
        pygame.display.flip()
        self.update_stream_frame()

    def draw_navigation_mode(self):
        self.screen.fill(self.navigation_bg)
        banner = pygame.Surface((self.width, self.height), pygame.SRCALPHA)
        pygame.draw.rect(
            banner,
            (*self.navigation_bg_dark, 160),
            pygame.Rect(50, 50, self.width - 100, self.height - 100),
            border_radius=36,
        )
        self.screen.blit(banner, (0, 0))

        headline = self.navigation_font.render("Let's go", True, self.navigation_text)
        subline = self.message_font.render(
            'Starting navigation.',
            True,
            self.navigation_text,
        )
        destination = self.pending_destination_label or 'your destination'
        detail = self.message_font.render(
            destination,
            True,
            self.navigation_text,
        )

        self.screen.blit(
            headline,
            headline.get_rect(center=(self.cx, self.height // 2 - 70)),
        )
        self.screen.blit(
            subline,
            subline.get_rect(center=(self.cx, self.height // 2 + 10)),
        )
        self.screen.blit(
            detail,
            detail.get_rect(center=(self.cx, self.height // 2 + 64)),
        )

    def process_destination(self):
        destination = self.input_buffer.strip()
        self.input_buffer = ''

        if destination.lower() == 'q':
            rclpy.shutdown()
            return

        if not destination:
            self.clear_override()
            return

        if self.navigation_mode_active:
            self.clear_override()
            return

        if self.awaiting_confirmation:
            self.handle_confirmation_response(destination)
            return

        if self.interpreter.is_conversation_end(destination):
            ending = self.interpreter.ending_response(destination)
            self.set_override(
                expression=ending['expression'],
                message=ending['message'],
            )
            self.state_timeout_at = self.now_seconds() + self.response_duration_sec
            return

        if self.pending_future is not None and not self.pending_future.done():
            self.set_override(
                expression='confused',
                message='One moment, I am still checking the last request.',
            )
            return

        self.set_override(
            expression='confused',
            message=f'I am looking up {destination} in the SEIC directory.',
        )
        self.pending_future = self.assistant_pool.submit(self.lookup_destination, destination)

    def show_robot_message(self, message: str):
        self.current_message = message
        self.speak_robot_message(message, save=True)

    def save_robot_message(self, message: str):
        if not message:
            return

        timestamp = datetime.now(timezone.utc).isoformat()
        with self.phrase_log_file.open('a', encoding='utf-8') as log_handle:
            log_handle.write(f'{timestamp} {message}\n')

        self.get_logger().info(f'Saved robot GUI message: {message}')

    def speak_robot_message(self, message: str, save=False):
        if not message:
            return

        if save:
            self.save_robot_message(message)

        if not self.speak_phrases or message == self.last_spoken_message:
            return

        self.last_spoken_message = message
        now = self.now_seconds()
        speech_duration = max(
            1.5,
            len(message) / max(5.0, self.speech_chars_per_sec),
        )
        self.speech_display_text = message
        self.speech_display_started_at = now
        self.speech_display_until = now + speech_duration + 0.8
        self.set_expression('happy')
        self.speech_queue.put(message)

    def speech_loop(self):
        try:
            import pyttsx3

            engine = pyttsx3.init()
            self.configure_speech_engine(engine)

            while not self.speech_stop.is_set():
                phrase = self.speech_queue.get()
                if phrase is None:
                    return
                engine.say(phrase)
                engine.runAndWait()
        except Exception as exc:
            self.get_logger().warning(f'Could not speak phrase: {exc}')

    def configure_speech_engine(self, engine):
        engine.setProperty('rate', self.speech_rate)
        engine.setProperty('volume', max(0.0, min(1.0, self.speech_volume)))

        if not self.speech_voice_id:
            return

        wanted = self.speech_voice_id.lower()
        for voice in engine.getProperty('voices'):
            voice_id = getattr(voice, 'id', '')
            voice_name = getattr(voice, 'name', '')
            if wanted in voice_id.lower() or wanted in voice_name.lower():
                engine.setProperty('voice', voice_id)
                self.get_logger().info(f'Using TTS voice: {voice_id}')
                return

        self.get_logger().warning(
            f'TTS voice "{self.speech_voice_id}" was not found. Using default voice.'
        )

    def load_stt_model(self):
        backend = self.stt_backend.strip().lower()
        try:
            if backend in ('openai_whisper', 'whisper'):
                self.load_openai_whisper_model()
            elif backend in ('faster_whisper', 'faster-whisper'):
                self.load_faster_whisper_model()
            else:
                raise ValueError(
                    'stt_backend must be "openai_whisper" or "faster_whisper"'
                )
        except Exception as exc:
            self.get_logger().error(
                f'Failed to load STT model: {exc}. '
                'Speech-to-text will be unavailable.'
            )
            self.stt_status = 'Speech-to-text model is unavailable.'
            self.stt_enabled = False

    def load_openai_whisper_model(self):
        if self.stt_model_path is None or not self.stt_model_path.is_file():
            raise FileNotFoundError(
                f'Local OpenAI Whisper model file not found: {self.stt_model_path}'
            )

        import whisper

        self.get_logger().info(
            'Loading local OpenAI Whisper model from '
            f'{self.stt_model_path} on {self.stt_device}...'
        )
        self.stt_model = whisper.load_model(
            self.stt_model_path.stem,
            device=self.stt_device,
            download_root=str(self.stt_model_path.parent),
        )
        self.stt_model_kind = 'openai_whisper'
        self.stt_status = 'Press Spacebar to speak.'
        self.get_logger().info('Local OpenAI Whisper model loaded.')

    def load_faster_whisper_model(self):
        from faster_whisper import WhisperModel

        model_source = (
            str(self.stt_model_path)
            if self.stt_model_path is not None and self.stt_model_path.is_dir()
            else self.stt_model_size
        )
        requested_device = self.stt_device.strip().lower()
        requested_compute_type = self.stt_compute_type.strip().lower()
        attempts = self.get_faster_whisper_load_attempts(
            requested_device,
            requested_compute_type,
        )

        last_error = None
        for device, compute_type in attempts:
            try:
                self.get_logger().info(
                    f'Loading faster-whisper model ({model_source}) on '
                    f'{device} with {compute_type} compute...'
                )
                self.stt_model = WhisperModel(
                    model_source,
                    device=device,
                    compute_type=compute_type,
                    local_files_only=self.stt_local_files_only,
                )
                self.stt_device = device
                self.stt_compute_type = compute_type
                self.stt_model_kind = 'faster_whisper'
                self.stt_status = 'Press Spacebar to speak.'
                self.get_logger().info(
                    'faster-whisper model loaded '
                    f'on {device} with {compute_type} compute.'
                )
                return
            except Exception as exc:
                last_error = exc
                self.get_logger().warning(
                    'Failed to load faster-whisper on '
                    f'{device} with {compute_type}: {exc}'
                )

        raise RuntimeError(
            f'Could not load faster-whisper model from {model_source}'
        ) from last_error

    def get_faster_whisper_load_attempts(
        self,
        requested_device,
        requested_compute_type,
    ):
        if requested_compute_type in ('', 'auto'):
            gpu_compute_type = 'int8_float16'
            cpu_compute_type = 'int8'
        else:
            gpu_compute_type = requested_compute_type
            cpu_compute_type = requested_compute_type

        if requested_device in ('', 'auto'):
            return [('cuda', gpu_compute_type), ('cpu', cpu_compute_type)]

        compute_type = (
            cpu_compute_type if requested_device == 'cpu' else gpu_compute_type
        )
        attempts = [(requested_device, compute_type)]
        if requested_device != 'cpu':
            attempts.append(('cpu', 'int8'))
        return attempts

    def start_stt_listening(self):
        if not self.stt_enabled:
            self.stt_status = 'Speech-to-text is unavailable.'
            return
        if self.stt_model is None:
            self.stt_status = 'Speech-to-text model is not loaded.'
            return
        if self.listening:
            return
        self.get_logger().info('STT: listening...')
        self.listening = True
        self.listening_until = self.now_seconds() + self.stt_timeout_sec
        self.stt_status = 'Listening... speak now.'
        self.stt_last_text = ''
        self.listening_thread = threading.Thread(
            target=self._stt_listen_loop,
            daemon=True,
        )
        self.listening_thread.start()

    def _stt_listen_loop(self):
        try:
            import sounddevice as sd
            import numpy as np

            num_channels = 1
            dtype = 'float32'
            frames_per_buffer = 2048

            with sd.InputStream(
                channels=num_channels,
                samplerate=self.stt_sample_rate,
                dtype=dtype,
                blocksize=frames_per_buffer,
            ) as stream:
                collected = []
                silence_start = None
                voice_started = False
                audio_timeout = self.now_seconds() + self.stt_timeout_sec

                while self.now_seconds() < audio_timeout:
                    chunk, _ = stream.read(frames_per_buffer)
                    audio_data = np.ascontiguousarray(chunk.squeeze())
                    collected.append(audio_data)

                    # Voice activity detection using energy threshold
                    energy = np.mean(audio_data ** 2)
                    if energy > 0.001:
                        voice_started = True
                        silence_start = None
                    else:
                        if not voice_started:
                            continue
                        elif silence_start is None:
                            silence_start = self.now_seconds()
                        elif (
                            self.now_seconds() - silence_start
                            >= self.stt_silence_timeout_sec
                        ):
                            break

            if not collected:
                self.get_logger().info('STT: no audio collected')
                self.stt_status = 'No audio was captured.'
                return

            audio_array = np.concatenate(collected, axis=0)

            text = self.transcribe_audio(audio_array)

            if text:
                self.get_logger().info(f'STT: recognized "{text}"')
                self.stt_result_queue.put(text[:40])
            else:
                self.get_logger().info('STT: no speech detected in audio')
                self.stt_status = 'No speech detected.'
        except Exception as exc:
            self.get_logger().warning(f'STT failed: {exc!r}')
            self.stt_status = f'Speech-to-text failed: {exc!r}'
        finally:
            self.listening = False
            self.listening_until = None

    def transcribe_audio(self, audio_array):
        if self.stt_model_kind == 'faster_whisper':
            segments, _ = self.stt_model.transcribe(
                audio_array,
                language='en',
                beam_size=5,
                vad_filter=True,
            )
            return ''.join(segment.text for segment in segments).strip()

        if self.stt_model_kind == 'openai_whisper':
            result = self.stt_model.transcribe(
                audio_array,
                language='en',
                fp16=self.stt_device != 'cpu',
            )
            return str(result.get('text', '')).strip()

        raise RuntimeError('No speech-to-text model is loaded.')

    def process_stt_results(self):
        while True:
            try:
                text = self.stt_result_queue.get_nowait()
            except queue.Empty:
                return

            self.input_buffer = text
            self.stt_last_text = text
            self.stt_status = f'Recognized: {text}'

    def update_speech_display(self):
        if not self.speech_display_text or self.speech_display_started_at is None:
            return

        now = self.now_seconds()
        elapsed = max(0.0, now - self.speech_display_started_at)
        visible_count = min(
            len(self.speech_display_text),
            int(elapsed * max(5.0, self.speech_chars_per_sec)) + 1,
        )
        self.current_message = self.speech_display_text[:visible_count]

        if self.speech_display_until is not None and now >= self.speech_display_until:
            self.speech_display_text = ''
            self.speech_display_started_at = None
            self.speech_display_until = None

    def lookup_destination(self, destination: str):
        target = self.interpreter.extract_target(destination)
        match = self.directory.find_best_match(target or destination)
        base_message = self.directory.build_response(match)
        expression = self.directory.expression_for_match(match)
        if match.entry is None:
            return {
                'message': base_message,
                'expression': expression,
                'await_confirmation': False,
                'destination_label': None,
            }

        destination_label = (
            match.entry.location if match.entry.kind == 'person'
            else match.entry.title
        )
        message = (
            f'{base_message} Would you like me to guide you to {destination_label}? '
            'Please type yes or no.'
        )
        return {
            'message': message,
            'expression': expression,
            'await_confirmation': True,
            'destination_label': destination_label,
        }

    def handle_confirmation_response(self, response: str):
        normalized = response.strip().lower()
        yes_tokens = {'yes', 'y', 'yeah', 'yep', 'sure', 'ok', 'okay'}
        no_tokens = {'no', 'n', 'nope', 'nah'}

        if normalized in yes_tokens:
            self.get_logger().info(
                f'Navigation mode accepted for: {self.pending_destination_label or "destination"}'
            )
            if self.pending_destination_label:
                label_msg = String()
                label_msg.data = self.pending_destination_label
                self.label_pub.publish(label_msg)
            self.navigation_mode_active = True
            self.awaiting_confirmation = False
            self.override_expression = None
            self.override_message = None
            self.override_until = None
            self.active_expression = 'happy'
            self.set_expression('happy')
            self.show_robot_message('Starting navigation.')
            self.state_timeout_at = None
            self.publish_light_state()
            return

        if normalized in no_tokens:
            self.set_override(
                expression='neutral',
                message='Okay. You can enter another person or room when you are ready.',
            )
            self.awaiting_confirmation = False
            self.pending_destination_label = None
            self.state_timeout_at = self.now_seconds() + self.response_duration_sec
            self.publish_light_state()
            return

        destination_label = self.pending_destination_label or 'that location'
        self.set_override(
            expression='confused',
            message=(
                'Please type yes or no. Would you like me to guide you to '
                f'{destination_label}?'
            ),
        )
        self.awaiting_confirmation = True
        self.state_timeout_at = self.now_seconds() + self.confirmation_timeout_sec
        self.publish_light_state()

    def now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def handle_key(self, key, mod=0):
        if key == pygame.K_RETURN:
            self.process_destination()
        elif key == pygame.K_BACKSPACE:
            self.input_buffer = self.input_buffer[:-1]
        elif key == pygame.K_h:
            self.show_help = not self.show_help
        elif key == pygame.K_SPACE:
            self.start_stt_listening()

    def update_frame(self):
        self.clock.tick(max(5.0, self.frame_rate))
        self.process_web_commands()

        if self.pending_future is not None and self.pending_future.done():
            try:
                reply = self.pending_future.result()
                self.set_override(expression=reply['expression'], message=reply['message'])
                self.pending_destination_label = reply['destination_label']
                self.awaiting_confirmation = reply['await_confirmation']
                if self.awaiting_confirmation:
                    self.state_timeout_at = self.now_seconds() + self.confirmation_timeout_sec
                self.publish_light_state()
            except Exception as exc:
                self.get_logger().warning(f'Destination lookup failed: {exc!r}')
                self.set_override(
                    expression='confused',
                    message='Sorry, I had trouble thinking of a reply just now.',
                )
                self.publish_light_state()
            finally:
                self.pending_future = None

        if (
            self.override_until is not None
            and self.now_seconds() >= self.override_until
        ):
            if not self.awaiting_confirmation and not self.navigation_mode_active:
                self.clear_override()

        if (
            self.state_timeout_at is not None
            and self.now_seconds() >= self.state_timeout_at
            and not self.navigation_mode_active
        ):
            self.clear_override()

        self.update_speech_display()
        self.process_stt_results()

        if self.listening and self.listening_until is not None:
            if self.now_seconds() >= self.listening_until:
                self.listening = False
                self.get_logger().info('STT: listening timeout expired')

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                rclpy.shutdown()
                return
            if event.type == pygame.KEYDOWN:
                if self.awaiting_confirmation or self.navigation_mode_active:
                    timeout_window = (
                        self.navigation_timeout_sec
                        if self.navigation_mode_active
                        else self.confirmation_timeout_sec
                    )
                    self.state_timeout_at = self.now_seconds() + timeout_window
                if event.key == pygame.K_ESCAPE:
                    rclpy.shutdown()
                    return
                if event.unicode and event.unicode.isprintable() and event.key not in (
                    pygame.K_RETURN,
                    pygame.K_BACKSPACE,
                    pygame.K_SPACE,
                ):
                    if len(self.input_buffer) < 40:
                        self.input_buffer += event.unicode
                self.handle_key(event.key, event.mod)

        self.draw()

    def destroy_node(self):
        self.assistant_pool.shutdown(wait=False)
        self.speech_stop.set()
        if self.speech_thread is not None:
            self.speech_queue.put(None)
            self.speech_thread.join(timeout=1.0)
        if self.stream_server is not None:
            self.stream_server.shutdown()
            self.stream_server.server_close()
        pygame.quit()
        super().destroy_node()


def main(args=None):
    libgomp_path = '/lib/aarch64-linux-gnu/libgomp.so.1'
    ld_preload = os.environ.get('LD_PRELOAD', '')
    if Path(libgomp_path).exists() and libgomp_path not in ld_preload:
        print(
            'Warning: LD_PRELOAD does not include '
            f'{libgomp_path}. If Whisper/Torch fails with "cannot allocate '
            'memory in static TLS block", launch this node with '
            f'LD_PRELOAD={libgomp_path}.',
            flush=True,
        )

    rclpy.init(args=args)
    node = FaceDisplayNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
        else:
            node.destroy_node()


if __name__ == '__main__':
    main()
