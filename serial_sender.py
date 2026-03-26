#!/usr/bin/env python3
"""
cmd_vel_keyboard_fixed.py

Fixed keyboard control with proper thread safety and graceful shutdown.
19-byte bridge-compatible radar protocol (vx, vy, wz, yaw_deg), plus A3 RobotControl frames and 0x5C/0x5D referee status packets.

Usage:
  python3 cmd_vel_keyboard_fixed.py --port /dev/pts/Y --keyboard --speed 1.0
  # »ò¸´ÖÆµ½ Desktop ×÷Îª serial_sender.py Ê¹ÓÃ

Controls:
  W/S: ×ó/ÓÒ  A/D: Ç°/ºó  Q/E: µ×ÅÌÐý×ª
  SPACE: ¼±Í£  ESC: ÍË³ö
"""

import argparse
import serial
import struct
import time
import sys
import threading

try:
    from pynput import keyboard
    PYNPUT_AVAILABLE = True
except ImportError:
    PYNPUT_AVAILABLE = False


RADAR_CMD_HEADER = b"\xA5\x5A"
RADAR_TELEM_HEADER = b"\xA6\x6A"
ROBOT_CTRL_HEADER = b"\xA3"
GAME_STATUS_HEADER = b"\x5C"
ROBOT_STATUS_HEADER = b"\x5D"
RADAR_FRAME_SIZE = 19
ROBOT_CTRL_FRAME_SIZE = 16
GAME_STATUS_FRAME_SIZE = 6
ROBOT_STATUS_FRAME_SIZE = 10

SENTRY_EXT_FLAG_SCAN_CONTROL_VALID = 0x01
SENTRY_EXT_FLAG_STOP_GIMBAL_SCAN = 0x02
SENTRY_EXT_FLAG_SCAN_ENABLED = 0x04
SENTRY_EXT_FLAG_ALLOW_VISION_CONTROL = 0x08
SENTRY_EXT_FLAG_SEARCH_WHEN_TARGET_LOST = 0x10


def crc8(data: bytes) -> int:
    """CRC8 for radar protocol (polynomial 0x07)."""
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def crc16_x25(data: bytes) -> int:
    """CRC16/X25 used by the RobotControl bridge frame."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0x8408
            else:
                crc >>= 1
    return crc & 0xFFFF


def encode_robot_control(
    control_flags: int,
    chassis_spin_vel: float = 0.0,
    scan_yaw_rate_deg_s: float = 0.0,
    search_pitch_deg: float = float("nan"),
) -> bytes:
    payload = struct.pack(
        "<BBfff",
        ROBOT_CTRL_HEADER[0],
        control_flags & 0xFF,
        chassis_spin_vel,
        scan_yaw_rate_deg_s,
        search_pitch_deg,
    )
    return payload + struct.pack('<H', crc16_x25(payload))


def robot_control_to_fields(msg) -> tuple[int, float, float, float]:
    stop_gimbal_scan = bool(getattr(msg, "stop_gimbal_scan", False))
    chassis_spin_vel = float(getattr(msg, "chassis_spin_vel", 0.0))
    scan_enabled = bool(getattr(msg, "scan_enabled", False))
    allow_vision_control = bool(getattr(msg, "allow_vision_control", False))
    search_when_target_lost = bool(getattr(msg, "search_when_target_lost", False))
    scan_yaw_rate_deg_s = float(getattr(msg, "scan_yaw_rate_deg_s", 0.0))
    search_pitch_deg = float(getattr(msg, "search_pitch_deg", 0.0))

    has_functional_fields = (
        scan_enabled
        or allow_vision_control
        or search_when_target_lost
        or abs(scan_yaw_rate_deg_s) > 1e-3
        or abs(search_pitch_deg) > 1e-3
    )

    control_flags = SENTRY_EXT_FLAG_SCAN_CONTROL_VALID
    if stop_gimbal_scan:
        control_flags |= SENTRY_EXT_FLAG_STOP_GIMBAL_SCAN

    if has_functional_fields:
        if scan_enabled:
            control_flags |= SENTRY_EXT_FLAG_SCAN_ENABLED
        if allow_vision_control:
            control_flags |= SENTRY_EXT_FLAG_ALLOW_VISION_CONTROL
        if search_when_target_lost:
            control_flags |= SENTRY_EXT_FLAG_SEARCH_WHEN_TARGET_LOST
        return (
            control_flags,
            chassis_spin_vel,
            scan_yaw_rate_deg_s,
            search_pitch_deg,
        )

    if stop_gimbal_scan:
        control_flags |= SENTRY_EXT_FLAG_ALLOW_VISION_CONTROL
    else:
        control_flags |= SENTRY_EXT_FLAG_SCAN_ENABLED

    return control_flags, chassis_spin_vel, 0.0, float("nan")


def encode_radar_cmd(vx: float, vy: float, wz: float,
                     gimbal_yaw_rad: float = 0.0, gimbal_pitch_rad: float = 0.0,
                     flip_x: bool = False, flip_y: bool = False) -> bytes:
    """
    19-byte bridge protocol: [0xA5][0x5A][vx:4][vy:4][wz:4][yaw_deg:4][CRC8:1]
    Ä¬ÈÏ£ºW=Ç° S=ºó A=×ó D=ÓÒ (vx=Ç°ºó vy=×óÓÒ£¬µ×ÅÌy+ÎªÓÒ¹ÊA×ó=-vy)
    flip_x: Ç°ºó·´ÁË  flip_y: ×óÓÒ·´ÁË

    `gimbal_yaw_rad` is kept for backward compatibility and converted to degrees
    for the bridge yaw field. `gimbal_pitch_rad` is not present in the 19-byte
    protocol and is therefore ignored by the sender.
    """
    vx_out = (-1 if flip_x else 1) * vx   # W=Ç° S=ºó
    vy_out = (-1 if flip_y else 1) * (-vy)  # A=×ó D=ÓÒ
    yaw_deg = gimbal_yaw_rad * 57.29577951308232
    payload = struct.pack('<2s4f', RADAR_CMD_HEADER, vx_out, vy_out, wz, yaw_deg)
    return payload + bytes([crc8(payload)])


def parse_radar_telem(frame: bytes):
    if len(frame) != RADAR_FRAME_SIZE or frame[:2] != RADAR_TELEM_HEADER:
        raise ValueError("invalid radar telemetry frame")
    if crc8(frame[:-1]) != frame[-1]:
        raise ValueError("invalid radar telemetry CRC8")
    _, vx, vy, wz, reserved0 = struct.unpack('<2s4f', frame[:-1])
    return vx, vy, wz, reserved0


def parse_game_status_frame(frame: bytes):
    if len(frame) != GAME_STATUS_FRAME_SIZE or frame[:1] != GAME_STATUS_HEADER:
        raise ValueError("invalid game status frame")
    if crc16_x25(frame[:-2]) != struct.unpack('<H', frame[-2:])[0]:
        raise ValueError("invalid game status CRC16")
    _header, game_progress, stage_remain_time = struct.unpack('<BBH', frame[:-2])
    return int(game_progress), int(stage_remain_time)


def parse_robot_status_frame(frame: bytes):
    if len(frame) != ROBOT_STATUS_FRAME_SIZE or frame[:1] != ROBOT_STATUS_HEADER:
        raise ValueError("invalid robot status frame")
    if crc16_x25(frame[:-2]) != struct.unpack('<H', frame[-2:])[0]:
        raise ValueError("invalid robot status CRC16")
    _header, robot_id, current_hp, shooter_heat, team_color, is_attacked = struct.unpack(
        '<BBHHBB', frame[:-2]
    )
    return (
        int(robot_id),
        int(current_hp),
        int(shooter_heat),
        bool(team_color),
        bool(is_attacked),
    )


class RadarBridgeParser:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data: bytes):
        events = []
        if data:
            self.buffer.extend(data)

        while len(self.buffer) >= 1:
            if len(self.buffer) >= 2:
                header = bytes(self.buffer[:2])
                if header == RADAR_TELEM_HEADER:
                    if len(self.buffer) < RADAR_FRAME_SIZE:
                        break
                    frame = bytes(self.buffer[:RADAR_FRAME_SIZE])
                    try:
                        events.append(("radar_telem", parse_radar_telem(frame)))
                        del self.buffer[:RADAR_FRAME_SIZE]
                    except ValueError:
                        del self.buffer[0]
                    continue

                if header == RADAR_CMD_HEADER:
                    if len(self.buffer) < RADAR_FRAME_SIZE:
                        break
                    del self.buffer[:RADAR_FRAME_SIZE]
                    continue

            head1 = self.buffer[0]
            if head1 == GAME_STATUS_HEADER[0]:
                if len(self.buffer) < GAME_STATUS_FRAME_SIZE:
                    break
                frame = bytes(self.buffer[:GAME_STATUS_FRAME_SIZE])
                try:
                    events.append(("game_status", parse_game_status_frame(frame)))
                    del self.buffer[:GAME_STATUS_FRAME_SIZE]
                except ValueError:
                    del self.buffer[0]
                continue

            if head1 == ROBOT_STATUS_HEADER[0]:
                if len(self.buffer) < ROBOT_STATUS_FRAME_SIZE:
                    break
                frame = bytes(self.buffer[:ROBOT_STATUS_FRAME_SIZE])
                try:
                    events.append(("robot_status", parse_robot_status_frame(frame)))
                    del self.buffer[:ROBOT_STATUS_FRAME_SIZE]
                except ValueError:
                    del self.buffer[0]
                continue

            del self.buffer[0]

        return events


class SerialForwarder:
    def __init__(self, port: str, baud: int = 115200, timeout=1.0,
                 flip_x: bool = False, flip_y: bool = False):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.flip_x = flip_x
        self.flip_y = flip_y
        self.ser = None
        self.lock = threading.Lock()
        self.last_reconnect_attempt = 0
        self.reconnect_interval = 2.0
        self.bridge_parser = RadarBridgeParser()
        self.on_game_status = None
        self.on_robot_status = None
        self._warned_pitch_drop = False

    def open(self):
        with self.lock:
            if self.ser and self.ser.is_open:
                return
            try:
                self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
                time.sleep(0.2)
                if not getattr(self, '_reader_run', False):
                    self._reader_run = True
                    self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
                    self._reader_thread.start()
                print(f"[INFO] Serial port {self.port} opened at {self.baud} baud.")
            except Exception as e:
                print(f"[ERROR] Failed to open serial port: {e}")
                self.ser = None

    def send(self, vx: float, vy: float, wz: float,
             gimbal_yaw_rad: float = 0.0, gimbal_pitch_rad: float = 0.0):
        if abs(gimbal_pitch_rad) > 1e-6 and not self._warned_pitch_drop:
            print("[WARN] 19-byte radar bridge protocol ignores pitch commands; dropping gimbal_pitch_rad.")
            self._warned_pitch_drop = True
        frame = encode_radar_cmd(vx, vy, wz, gimbal_yaw_rad, gimbal_pitch_rad,
                                 self.flip_x, self.flip_y)
        if not self.ser or not self.ser.is_open:
            now = time.time()
            if now - self.last_reconnect_attempt > self.reconnect_interval:
                self.last_reconnect_attempt = now
                self.open()
            return False
        try:
            with self.lock:
                self.ser.write(frame)
            return True
        except Exception as e:
            print(f"[ERROR] Serial write failed: {e}")
            self.close()
            return False

    def send_robot_control(
        self,
        control_flags: int,
        chassis_spin_vel: float = 0.0,
        scan_yaw_rate_deg_s: float = 0.0,
        search_pitch_deg: float = float("nan"),
    ):
        frame = encode_robot_control(
            control_flags,
            chassis_spin_vel,
            scan_yaw_rate_deg_s,
            search_pitch_deg,
        )
        if not self.ser or not self.ser.is_open:
            now = time.time()
            if now - self.last_reconnect_attempt > self.reconnect_interval:
                self.last_reconnect_attempt = now
                self.open()
            return False
        try:
            with self.lock:
                self.ser.write(frame)
            return True
        except Exception as e:
            print(f"[ERROR] RobotControl write failed: {e}")
            self.close()
            return False

    def close(self):
        with self.lock:
            self._reader_run = False
            if self.ser:
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None

    def _reader_loop(self):
        while getattr(self, '_reader_run', False):
            if not self.ser or not self.ser.is_open:
                time.sleep(1.0)
                continue
            try:
                chunk = self.ser.read(max(getattr(self.ser, 'in_waiting', 0), 1))
                if not chunk:
                    continue

                events = self.bridge_parser.feed(chunk)
                for kind, payload in events:
                    if kind == "radar_telem":
                        vx, vy, wz, reserved0 = payload
                        print(
                            f"[RADAR TELEM] vx={vx:+.3f} vy={vy:+.3f} wz={wz:+.3f} reserved={reserved0:+.3f}"
                        )
                    elif kind == "game_status":
                        if self.on_game_status is not None:
                            try:
                                self.on_game_status(*payload)
                            except Exception:
                                pass
                    elif kind == "robot_status":
                        if self.on_robot_status is not None:
                            try:
                                self.on_robot_status(*payload)
                            except Exception:
                                pass

                if (
                    events
                    or (len(chunk) >= 2 and bytes(chunk[:2]) in (RADAR_CMD_HEADER, RADAR_TELEM_HEADER))
                    or (len(chunk) >= 1 and chunk[0] in (GAME_STATUS_HEADER[0], ROBOT_STATUS_HEADER[0], ROBOT_CTRL_HEADER[0]))
                ):
                    continue

                try:
                    s = chunk.decode('ascii', errors='ignore').rstrip('\r\n')
                    if s.strip():
                        print(f'[STM32] {s}')
                except Exception:
                    pass
            except Exception:
                time.sleep(0.05)


def keyboard_run(forwarder, move_speed=0.3, turn_speed=1.0, send_rate=100):
    """Keyboard control with proper thread safety and smooth acceleration."""

    if not PYNPUT_AVAILABLE:
        print("[ERROR] pynput library not found. Install with: pip install pynput")
        sys.exit(1)

    print("\n" + "=" * 50)
    print(" KEYBOARD CONTROL MODE (Swerve Chassis)")
    print("=" * 50)
    print(" W/S: Ç°/ºó  A/D: ×ó/ÓÒ  Q/E: µ×ÅÌÐý×ª")
    print(" SPACE: ¼±Í£  ESC: ÍË³ö")
    print("=" * 50 + "\n")

    RADAR_SMOOTH_ALPHA = 0.20
    RADAR_MAX_DELTA_V = 0.05
    RADAR_MAX_DELTA_W = 0.10

    vel_lock = threading.Lock()
    target_vel = {'vx': 0.0, 'vy': 0.0, 'wz': 0.0}
    filtered_vel = {'vx': 0.0, 'vy': 0.0, 'wz': 0.0}
    shutdown_event = threading.Event()
    pressed_keys = set()

    def update_velocity():
        with vel_lock:
            if 'w' in pressed_keys:
                target_vel['vx'] = move_speed
            elif 's' in pressed_keys:
                target_vel['vx'] = -move_speed
            else:
                target_vel['vx'] = 0.0

            if 'a' in pressed_keys:
                target_vel['vy'] = move_speed
            elif 'd' in pressed_keys:
                target_vel['vy'] = -move_speed
            else:
                target_vel['vy'] = 0.0

            if 'q' in pressed_keys:
                target_vel['wz'] = turn_speed
            elif 'e' in pressed_keys:
                target_vel['wz'] = -turn_speed
            else:
                target_vel['wz'] = 0.0

    def on_press(key):
        try:
            char = key.char.lower()
            if char in ['w', 'a', 's', 'd', 'q', 'e']:
                pressed_keys.add(char)
                update_velocity()
        except AttributeError:
            if key == keyboard.Key.space:
                with vel_lock:
                    target_vel['vx'] = 0.0
                    target_vel['vy'] = 0.0
                    target_vel['wz'] = 0.0
                print("[WARN] Emergency Stop!")
                for _ in range(5):
                    forwarder.send(0.0, 0.0, 0.0)
                    time.sleep(0.01)
            elif key == keyboard.Key.esc:
                print("[INFO] ESC pressed, shutting down...")
                shutdown_event.set()
                return False

    def on_release(key):
        try:
            char = key.char.lower()
            if char in pressed_keys:
                pressed_keys.discard(char)
                update_velocity()
        except AttributeError:
            pass

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    print("[INFO] Sending initial stop command...")
    for _ in range(5):
        forwarder.send(0.0, 0.0, 0.0)
        time.sleep(0.01)
    time.sleep(0.2)

    interval = 1.0 / max(1.0, send_rate)
    last_print = time.time()
    last_vel = {'vx': 0.0, 'vy': 0.0, 'wz': 0.0}

    print(f"[INFO] Starting send loop at {send_rate} Hz")

    try:
        while not shutdown_event.is_set():
            with vel_lock:
                target = target_vel.copy()

            lp_vx = filtered_vel['vx'] + RADAR_SMOOTH_ALPHA * (target['vx'] - filtered_vel['vx'])
            lp_vy = filtered_vel['vy'] + RADAR_SMOOTH_ALPHA * (target['vy'] - filtered_vel['vy'])
            lp_wz = filtered_vel['wz'] + RADAR_SMOOTH_ALPHA * (target['wz'] - filtered_vel['wz'])

            dvx = lp_vx - filtered_vel['vx']
            if dvx > RADAR_MAX_DELTA_V:
                dvx = RADAR_MAX_DELTA_V
            elif dvx < -RADAR_MAX_DELTA_V:
                dvx = -RADAR_MAX_DELTA_V
            filtered_vel['vx'] = filtered_vel['vx'] + dvx

            dvy = lp_vy - filtered_vel['vy']
            if dvy > RADAR_MAX_DELTA_V:
                dvy = RADAR_MAX_DELTA_V
            elif dvy < -RADAR_MAX_DELTA_V:
                dvy = -RADAR_MAX_DELTA_V
            filtered_vel['vy'] = filtered_vel['vy'] + dvy

            dwz = lp_wz - filtered_vel['wz']
            if dwz > RADAR_MAX_DELTA_W:
                dwz = RADAR_MAX_DELTA_W
            elif dwz < -RADAR_MAX_DELTA_W:
                dwz = -RADAR_MAX_DELTA_W
            filtered_vel['wz'] = filtered_vel['wz'] + dwz

            forwarder.send(filtered_vel['vx'], filtered_vel['vy'], filtered_vel['wz'])

            now = time.time()
            if (filtered_vel != last_vel) or (now - last_print > 0.5):
                vel_magnitude = (filtered_vel['vx'] ** 2 + filtered_vel['vy'] ** 2) ** 0.5
                print(
                    f"[CMD] vx={filtered_vel['vx']:+.2f} vy={filtered_vel['vy']:+.2f} wz={filtered_vel['wz']:+.2f} mag={vel_magnitude:.2f}",
                    end='\r',
                )
                last_print = now
                last_vel = filtered_vel.copy()

            time.sleep(interval)

    except KeyboardInterrupt:
        print("\n[INFO] Keyboard interrupt received")
        shutdown_event.set()

    finally:
        listener.stop()
        listener.join(timeout=1.0)

        print("\n[INFO] Sending final stop command...")
        for _ in range(10):
            success = forwarder.send(0.0, 0.0, 0.0)
            if not success:
                break
            time.sleep(0.02)

        time.sleep(0.2)
        forwarder.close()
        print("[INFO] Shutdown complete.")


def ros2_run(forwarder, topic='/cmd_vel', robot_control_topic='/robot_control', disable_status_pub=False):
    """ROS2 subscriber mode - forward Twist and RobotControl messages to serial."""
    try:
        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import Twist
        try:
            from rm_decision_interfaces.msg import GameStatus, RobotControl, RobotStatus
        except Exception:
            GameStatus = None
            RobotControl = None
            RobotStatus = None
    except Exception as e:
        print(f"[ERROR] ROS2 import failed: {e}")
        print("[ERROR] Install ROS2 or run in keyboard mode")
        sys.exit(1)

    class CmdVelNode(Node):
        def __init__(self, forwarder):
            super().__init__('cmd_vel_forwarder')
            self.forwarder = forwarder
            self.subscription = self.create_subscription(Twist, topic, self.cb_twist, 10)
            self.robot_control_subscription = None
            self.game_status_pub = None
            self.robot_status_pub = None
            self.get_logger().info(f'[ROS2] Subscribed to {topic}')

            if RobotControl is not None:
                self.robot_control_subscription = self.create_subscription(
                    RobotControl, robot_control_topic, self.cb_robot_control, 10
                )
                self.get_logger().info(f'[ROS2] Subscribed to {robot_control_topic}')
            else:
                self.get_logger().warning(
                    '[ROS2] rm_decision_interfaces not available; RobotControl forwarding disabled'
                )

            if (not disable_status_pub) and GameStatus is not None and RobotStatus is not None:
                self.game_status_pub = self.create_publisher(GameStatus, '/game_status', 1)
                self.robot_status_pub = self.create_publisher(RobotStatus, '/robot_status', 10)
                self.forwarder.on_game_status = self.publish_game_status
                self.forwarder.on_robot_status = self.publish_robot_status
                self.get_logger().info('[ROS2] Publishing /game_status and /robot_status from bridge telemetry')
            elif disable_status_pub:
                self.get_logger().info('[ROS2] Status publishing disabled by flag; not publishing /game_status or /robot_status')
            else:
                self.get_logger().warning(
                    '[ROS2] rm_decision_interfaces not available; referee status publishing disabled'
                )

        def cb_twist(self, msg: Twist):
            vx = float(msg.linear.x)
            vy = float(msg.linear.y)
            wz = float(msg.angular.z)
            self.forwarder.send(vx, vy, wz)
            vel_magnitude = (vx ** 2 + vy ** 2) ** 0.5
            print(
                f"[NAV2 -> STM32] vx={vx:+.3f} vy={vy:+.3f} wz={wz:+.3f} mag={vel_magnitude:.2f}    ",
                end='\r',
            )

        def cb_robot_control(self, msg):
            (
                control_flags,
                chassis_spin_vel,
                scan_yaw_rate_deg_s,
                search_pitch_deg,
            ) = robot_control_to_fields(msg)
            self.forwarder.send_robot_control(
                control_flags,
                chassis_spin_vel,
                scan_yaw_rate_deg_s,
                search_pitch_deg,
            )

        def publish_game_status(self, game_progress: int, stage_remain_time: int):
            if self.game_status_pub is None:
                return
            msg = GameStatus()
            msg.game_progress = int(game_progress)
            msg.stage_remain_time = int(stage_remain_time)
            self.game_status_pub.publish(msg)

        def publish_robot_status(
            self,
            robot_id: int,
            current_hp: int,
            shooter_heat: int,
            team_color: bool,
            is_attacked: bool,
        ):
            if self.robot_status_pub is None:
                return
            msg = RobotStatus()
            msg.robot_id = int(robot_id)
            msg.current_hp = int(current_hp)
            msg.shooter_heat = int(shooter_heat)
            msg.team_color = bool(team_color)
            msg.is_attacked = bool(is_attacked)
            self.robot_status_pub.publish(msg)

    rclpy.init()
    node = CmdVelNode(forwarder)
    try:
        print(f"[INFO] Running ROS2 subscriber mode. Listening on {topic}")
        print(f"[INFO] RobotControl topic: {robot_control_topic}")
        print("[INFO] Press Ctrl+C to exit")
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by user")
    finally:
        print("[INFO] Sending final stop command...")
        forwarder.on_game_status = None
        forwarder.on_robot_status = None
        for _ in range(10):
            forwarder.send(0.0, 0.0, 0.0)
            time.sleep(0.02)
        node.destroy_node()
        rclpy.shutdown()
        forwarder.close()


def oneshot_run(forwarder, vx, vy, wz, duration, rate):
    """One-shot mode - send fixed velocity commands for specified duration."""

    RADAR_SMOOTH_ALPHA = 0.20
    RADAR_MAX_DELTA_V = 0.05
    RADAR_MAX_DELTA_W = 0.10

    filtered_vel = {'vx': 0.0, 'vy': 0.0, 'wz': 0.0}
    target_vel = {'vx': vx, 'vy': vy, 'wz': wz}

    interval = 1.0 / max(1.0, rate)
    num_frames = int(duration * rate)

    print(f"\n[INFO] One-shot mode:")
    print(f"  Target: vx={vx:.3f} vy={vy:.3f} wz={wz:.3f}")
    print(f"  Duration: {duration:.2f}s, Rate: {rate}Hz, Frames: {num_frames}")
    print("[INFO] Sending velocity commands...\n")

    print("[INFO] Sending initial stop...")
    for _ in range(5):
        forwarder.send(0.0, 0.0, 0.0)
        time.sleep(0.01)
    time.sleep(0.2)

    try:
        for frame_idx in range(num_frames):
            lp_vx = filtered_vel['vx'] + RADAR_SMOOTH_ALPHA * (target_vel['vx'] - filtered_vel['vx'])
            lp_vy = filtered_vel['vy'] + RADAR_SMOOTH_ALPHA * (target_vel['vy'] - filtered_vel['vy'])
            lp_wz = filtered_vel['wz'] + RADAR_SMOOTH_ALPHA * (target_vel['wz'] - filtered_vel['wz'])

            dvx = lp_vx - filtered_vel['vx']
            if dvx > RADAR_MAX_DELTA_V:
                dvx = RADAR_MAX_DELTA_V
            elif dvx < -RADAR_MAX_DELTA_V:
                dvx = -RADAR_MAX_DELTA_V
            filtered_vel['vx'] = filtered_vel['vx'] + dvx

            dvy = lp_vy - filtered_vel['vy']
            if dvy > RADAR_MAX_DELTA_V:
                dvy = RADAR_MAX_DELTA_V
            elif dvy < -RADAR_MAX_DELTA_V:
                dvy = -RADAR_MAX_DELTA_V
            filtered_vel['vy'] = filtered_vel['vy'] + dvy

            dwz = lp_wz - filtered_vel['wz']
            if dwz > RADAR_MAX_DELTA_W:
                dwz = RADAR_MAX_DELTA_W
            elif dwz < -RADAR_MAX_DELTA_W:
                dwz = -RADAR_MAX_DELTA_W
            filtered_vel['wz'] = filtered_vel['wz'] + dwz

            forwarder.send(filtered_vel['vx'], filtered_vel['vy'], filtered_vel['wz'])

            if (frame_idx + 1) % max(1, rate // 5) == 0:
                elapsed = (frame_idx + 1) * interval
                print(
                    f"  [{frame_idx + 1}/{num_frames}] {elapsed:.2f}s: "
                    f"vx={filtered_vel['vx']:+.3f} vy={filtered_vel['vy']:+.3f} wz={filtered_vel['wz']:+.3f}"
                )

            time.sleep(interval)

        print("\n[INFO] Sending stop command...")
        for _ in range(10):
            forwarder.send(0.0, 0.0, 0.0)
            time.sleep(0.02)

        print("[INFO] Done!")

    except KeyboardInterrupt:
        print("\n[WARN] Interrupted by user, sending stop...")
        for _ in range(10):
            forwarder.send(0.0, 0.0, 0.0)
            time.sleep(0.02)


def main():
    parser = argparse.ArgumentParser(
        description='Swerve chassis control (keyboard, ROS2, or one-shot velocity)'
    )
    parser.add_argument('--port', required=True, help='Serial port (COM9, /dev/pts/Y, etc.)')
    parser.add_argument('--baud', type=int, default=115200)

    parser.add_argument('--keyboard', action='store_true', help='Enable keyboard control mode')
    parser.add_argument('--ros2', action='store_true', help='Enable ROS2 Twist + RobotControl subscriber mode')

    parser.add_argument('--vx', type=float, help='X velocity (m/s) - enables one-shot mode')
    parser.add_argument('--vy', type=float, default=0.0, help='Y velocity (m/s)')
    parser.add_argument('--wz', type=float, default=0.0, help='Angular velocity (rad/s)')
    parser.add_argument('--duration', type=float, default=1.0, help='Duration in seconds (for one-shot mode)')

    parser.add_argument('--topic', default='/cmd_vel', help='ROS2 Twist topic name (default: /cmd_vel)')
    parser.add_argument('--robot-control-topic', default='/robot_control', help='ROS2 RobotControl topic name (default: /robot_control)')
    parser.add_argument('--disable-status-pub', action='store_true', help='Disable publishing /game_status and /robot_status from bridge telemetry')

    parser.add_argument('--speed', type=float, default=0.3, help='Movement speed for keyboard (0.0-1.0)')
    parser.add_argument('--rate', type=int, default=200, help='Send rate in Hz')
    parser.add_argument('--flip-x', dest='flip_x', action='store_true', help='Ç°ºó·´ÁËÊ±ÓÃ')
    parser.add_argument('--flip-y', dest='flip_y', action='store_true', help='×óÓÒ·´ÁËÊ±ÓÃ')

    args = parser.parse_args()

    fwd = SerialForwarder(args.port, args.baud, flip_x=args.flip_x, flip_y=args.flip_y)

    try:
        fwd.open()
        if fwd.ser is None:
            sys.exit(1)

        if args.vx is not None:
            oneshot_run(fwd, args.vx, args.vy, args.wz, args.duration, args.rate)
        elif args.keyboard:
            if args.speed <= 0 or args.speed > 1.0:
                print("[ERROR] --speed must be between 0 and 1.0")
                sys.exit(1)
            keyboard_run(fwd, move_speed=args.speed, send_rate=args.rate)
        elif args.ros2:
            ros2_run(
                fwd,
                topic=args.topic,
                robot_control_topic=args.robot_control_topic,
                disable_status_pub=args.disable_status_pub,
            )
        else:
            print("[ERROR] Please specify a mode:")
            print("\n[USAGE 1] One-shot velocity:")
            print("  python3 cmd_vel_keyboard_fixed.py --port /dev/pts/Y --vx 0.5 --vy 0.2 --duration 2.0")
            print("\n[USAGE 2] Keyboard control (¿É¸´ÖÆµ½ Desktop ×÷ serial_sender.py):")
            print("  python3 cmd_vel_keyboard_fixed.py --port /dev/pts/Y --keyboard --speed 1.0")
            print("\n[USAGE 3] ROS2 subscriber:")
            print("  python3 cmd_vel_keyboard_fixed.py --port /dev/pts/Y --ros2")
            sys.exit(1)
    finally:
        fwd.close()


if __name__ == '__main__':
    main()
