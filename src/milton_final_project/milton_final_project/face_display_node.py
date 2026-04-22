from dataclasses import dataclass
import math
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import textwrap

from ament_index_python.packages import get_package_share_directory
import pygame
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .robot_interpreter import RobotInterpreter
from .seic_directory import SeicDirectory


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
        self.declare_parameter('initial_expression', 'neutral')
        self.declare_parameter(
            'waiting_message',
            "Hi, I'm the navigation robot that helps you find a location or room.",
        )
        self.declare_parameter('response_duration_sec', 10.0)
        self.declare_parameter('confirmation_timeout_sec', 15.0)
        self.declare_parameter('navigation_timeout_sec', 20.0)
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
        self.expression_index = 0

        pygame.init()
        pygame.font.init()

        display_flags = pygame.FULLSCREEN if self.get_parameter('fullscreen').value else 0
        self.screen = pygame.display.set_mode((self.width, self.height), display_flags)
        self.clock = pygame.time.Clock()

        self.bg = (8, 10, 14)
        self.label = (225, 232, 244)
        self.help_color = (168, 178, 198)
        self.message_bg = (12, 18, 28, 235)
        self.message_outline = (66, 129, 164)
        self.message_text = (240, 244, 250)
        self.navigation_bg = (28, 142, 77)
        self.navigation_bg_dark = (16, 84, 46)
        self.navigation_text = (244, 255, 247)

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
            'neutral': AssetExpression('neutral', 'Neutral', 'bored', 124, 104, 48),
            'happy': AssetExpression('happy', 'Happy', 'happy', 58, 108, 6),
            'confused': AssetExpression('confused', 'Confused', 'thinking', 118, 118, 56),
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
        self.interpreter = RobotInterpreter()
        self.directory = SeicDirectory()

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

        self.timer = self.create_timer(1.0 / 30.0, self.update_frame)
        self.get_logger().info(f'Face monitor ready with assets from: {self.assets_dir}')

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
            self.current_message = message

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

    def expression_callback(self, msg: String):
        expression = msg.data.strip()
        if expression:
            self.set_override(expression=expression)

    def message_callback(self, msg: String):
        message = msg.data.strip()
        if message:
            self.set_override(message=message)

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
        shadow = pygame.Surface((self.face_size + 44, self.face_size + 44), pygame.SRCALPHA)
        pygame.draw.ellipse(shadow, (0, 0, 0, 72), shadow.get_rect())
        shadow_rect = shadow.get_rect(center=(self.cx, self.cy + 12 + face_offset_y))
        self.screen.blit(shadow, shadow_rect)

        face = pygame.transform.smoothscale(self.background, (self.face_size, self.face_size))
        face_rect = face.get_rect(center=(self.cx, self.cy + face_offset_y))
        self.screen.blit(face, face_rect)

    def draw_bottom_panel(self):
        panel_surface = pygame.Surface((self.width, self.bottom_panel_height), pygame.SRCALPHA)
        pygame.draw.rect(
            panel_surface,
            (6, 10, 18, 232),
            panel_surface.get_rect(),
            border_radius=28,
        )
        pygame.draw.line(
            panel_surface,
            (35, 58, 84, 255),
            (42, 0),
            (self.width - 42, 0),
            2,
        )
        panel_rect = panel_surface.get_rect(midbottom=(self.cx, self.height))
        self.screen.blit(panel_surface, panel_rect)

    def draw_expression(self):
        expr = self.current_expression()
        active_expr = expr
        eyes = self.loaded_eyes[active_expr.name]
        left_eye = self.scaled_surface(eyes['left'], active_expr.eye_height)
        right_eye = self.scaled_surface(eyes['right'], active_expr.eye_height)

        baseline = self.cy + active_expr.baseline_offset + self.current_face_offset_y()
        gaze_offset = self.current_gaze_offset()
        left_rect = left_eye.get_rect(
            midbottom=(self.cx - active_expr.eye_gap + gaze_offset, baseline)
        )
        right_rect = right_eye.get_rect(
            midbottom=(self.cx + active_expr.eye_gap + gaze_offset, baseline)
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
            (10, 14, 22, 240),
            box_surface.get_rect(),
            border_radius=18,
        )
        pygame.draw.rect(
            box_surface,
            (92, 173, 226),
            box_surface.get_rect(),
            width=2,
            border_radius=18,
        )

        box_rect = box_surface.get_rect(midbottom=(self.cx, self.height - 8))
        self.screen.blit(box_surface, box_rect)

        if self.awaiting_confirmation:
            placeholder = 'Type yes or no...'
        elif self.navigation_mode_active:
            placeholder = 'Press Enter to start over...'
        else:
            placeholder = 'Room or location...'

        display_text = self.input_buffer if self.input_buffer else placeholder
        text_color = self.message_text if self.input_buffer else (140, 149, 168)
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

    def draw(self):
        if self.navigation_mode_active:
            self.draw_navigation_mode()
            pygame.display.flip()
            return

        self.screen.fill(self.bg)
        self.draw_background()
        self.draw_expression()
        self.draw_bottom_panel()
        self.draw_message()
        self.draw_input_box()
        self.draw_status()
        pygame.display.flip()

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
            'Going to navigation mode.',
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
            self.set_override(expression='confused', message='I am still processing the last request.')
            return

            self.set_override(expression='confused', message=f'Looking up {destination} in the SEIC directory.')
        self.pending_future = self.assistant_pool.submit(self.lookup_destination, destination)

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

        destination_label = match.entry.location if match.entry.kind == 'person' else match.entry.title
        message = (
            f'{base_message} Do you need help getting to {destination_label}? '
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
            self.navigation_mode_active = True
            self.awaiting_confirmation = False
            self.override_expression = None
            self.override_message = None
            self.override_until = None
            self.active_expression = 'happy'
            self.set_expression('happy')
            self.current_message = 'Going to navigation mode.'
            self.state_timeout_at = self.now_seconds() + self.navigation_timeout_sec
            return

        if normalized in no_tokens:
            self.set_override(
                expression='neutral',
                message='Okay. If you need anything else, ask me about another room or person.',
            )
            self.awaiting_confirmation = False
            self.pending_destination_label = None
            self.state_timeout_at = self.now_seconds() + self.response_duration_sec
            return

        destination_label = self.pending_destination_label or 'that location'
        self.set_override(
            expression='confused',
            message=f'Please answer yes or no. Do you need help getting to {destination_label}?',
        )
        self.awaiting_confirmation = True
        self.state_timeout_at = self.now_seconds() + self.confirmation_timeout_sec

    def now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def handle_key(self, key):
        if key == pygame.K_RETURN:
            self.process_destination()
        elif key == pygame.K_BACKSPACE:
            self.input_buffer = self.input_buffer[:-1]
        elif key == pygame.K_LEFT:
            self.cycle_expression(-1)
        elif key == pygame.K_RIGHT:
            self.cycle_expression(1)
        elif key == pygame.K_h:
            self.show_help = not self.show_help

    def update_frame(self):
        self.clock.tick(30)

        if self.pending_future is not None and self.pending_future.done():
            try:
                reply = self.pending_future.result()
                self.set_override(expression=reply['expression'], message=reply['message'])
                self.pending_destination_label = reply['destination_label']
                self.awaiting_confirmation = reply['await_confirmation']
                if self.awaiting_confirmation:
                    self.state_timeout_at = self.now_seconds() + self.confirmation_timeout_sec
            except Exception as exc:
                self.get_logger().warning(f'Assistant reply failed: {exc}')
                self.set_override(
                    expression='confused',
                    message='Sorry, I had trouble thinking of a reply just now.',
                )
            finally:
                self.pending_future = None

        if (
            self.override_until is not None
            and self.now_seconds() >= self.override_until
        ):
            if not self.awaiting_confirmation and not self.navigation_mode_active:
                self.clear_override()

        if self.state_timeout_at is not None and self.now_seconds() >= self.state_timeout_at:
            self.clear_override()

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
                ):
                    if len(self.input_buffer) < 40:
                        self.input_buffer += event.unicode
                self.handle_key(event.key)

        self.draw()

    def destroy_node(self):
        self.assistant_pool.shutdown(wait=False)
        pygame.quit()
        super().destroy_node()


def main(args=None):
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
