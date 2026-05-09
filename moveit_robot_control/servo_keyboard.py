#!/usr/bin/env python3
"""
Keyboard teleoperation for MoveIt Servo (FR3).

  Arrow / . ;   — Cartesian linear (x / y / z)
  a d f v x c  — Rotation (yaw / pitch / roll)
  1 – 7         — Joint jog
  r             — Reverse joint direction
  j / t         — Switch to Joint / Twist mode
  w / e         — Planning frame / EE frame
  s             — Stop immediately
  q             — Quit

Tap: motion stops shortly after you release (no new motion key). Hold: OS key-repeat
must refresh commands; if hold does nothing, raise STOP_TIMEOUT toward ~0.55 s or
speed up repeat (`xset r rate 200 30`).
"""
import sys
import select
import tty
import termios
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from moveit_msgs.srv import ServoCommandType

PLANNING_FRAME = 'fr3_link0'
EE_FRAME       = 'fr3_hand'
JOINT_NAMES    = [f'fr3_joint{i}' for i in range(1, 8)]
RATE_HZ        = 50
# Idle after last motion key before zeroing velocity (see module docstring).
STOP_TIMEOUT   = 0.45
LINEAR_SPEED   = 0.1   # m/s
ANGULAR_SPEED  = 0.5   # rad/s
JOINT_SPEED    = 0.3   # rad/s


class ServoKeyboard(Node):
    def __init__(self):
        super().__init__('servo_keyboard')
        self._tp = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self._jp = self.create_publisher(JointJog,     '/servo_node/delta_joint_cmds',  10)
        self._sw = self.create_client(ServoCommandType, '/servo_node/switch_command_type')

        self._lock  = threading.Lock()
        self._frame = PLANNING_FRAME
        self._jvel  = JOINT_SPEED
        self._tw    = TwistStamped()
        self._jog   = JointJog()
        self._jog.joint_names = JOINT_NAMES
        self._jog.velocities  = [0.0] * len(JOINT_NAMES)
        self._ta = self._ja = False
        self._t0 = self.get_clock().now()

        self.create_timer(1.0 / RATE_HZ, self._tick)

    # ── 50 Hz publish loop ────────────────────────────────────────────────

    def _tick(self):
        with self._lock:
            now = self.get_clock().now()
            dt = (now - self._t0).nanoseconds * 1e-9
            if dt > STOP_TIMEOUT:
                if self._ta or self._ja:
                    self._zero()
                self._t0 = now
            if self._ta:
                self._tw.header.stamp = now.to_msg()
                self._tp.publish(self._tw)
            if self._ja:
                self._jog.header.stamp = now.to_msg()
                self._jp.publish(self._jog)

    def _zero(self):
        l = self._tw.twist.linear
        a = self._tw.twist.angular
        l.x = l.y = l.z = a.x = a.y = a.z = 0.0
        self._jog.velocities = [0.0] * len(JOINT_NAMES)
        self._ta = self._ja = False

    # ── key dispatch ──────────────────────────────────────────────────────

    # Maps key label → (linear/angular object attr, sign)
    _TWIST_MAP = {
        'UP':    ('linear',  'x',  1), 'DOWN':  ('linear',  'x', -1),
        'RIGHT': ('linear',  'y',  1), 'LEFT':  ('linear',  'y', -1),
        ';':     ('linear',  'z',  1), '.':     ('linear',  'z', -1),
        'd':     ('angular', 'z',  1), 'a':     ('angular', 'z', -1),
        'f':     ('angular', 'y',  1), 'v':     ('angular', 'y', -1),
        'c':     ('angular', 'x',  1), 'x':     ('angular', 'x', -1),
    }

    def on_key(self, key):
        switch_mode = None
        with self._lock:
            self._tw.header.frame_id = self._frame

            if key in self._TWIST_MAP:
                part, attr, sign = self._TWIST_MAP[key]
                self._zero()
                speed = LINEAR_SPEED if part == 'linear' else ANGULAR_SPEED
                setattr(getattr(self._tw.twist, part), attr, sign * speed)
                self._ta = True
                self._t0 = self.get_clock().now()

            elif key in '1234567':
                self._zero()
                self._jog.velocities[int(key) - 1] = self._jvel
                self._ja = True
                self._t0 = self.get_clock().now()

            elif key == 'r':
                self._jvel *= -1
                print(f'Joint direction reversed ({self._jvel:+.1f} rad/s)')

            elif key == 's':
                self._zero()

            elif key == 'j':
                switch_mode = ServoCommandType.Request.JOINT_JOG

            elif key == 't':
                switch_mode = ServoCommandType.Request.TWIST

            elif key == 'w':
                self._frame = PLANNING_FRAME
                print(f'Frame → {self._frame}')

            elif key == 'e':
                self._frame = EE_FRAME
                print(f'Frame → {self._frame}')

        if switch_mode is not None:
            self._switch_mode(switch_mode)

    def _switch_mode(self, mode):
        if not self._sw.wait_for_service(timeout_sec=1.0):
            print('switch_command_type service not available')
            return
        req = ServoCommandType.Request()
        req.command_type = mode
        self._sw.call_async(req)
        label = 'Twist' if mode == ServoCommandType.Request.TWIST else 'JointJog'
        print(f'Mode → {label}')


# ── keyboard reading ──────────────────────────────────────────────────────

def read_key():
    ch = sys.stdin.read(1)
    if ch == '\x1b':
        if sys.stdin.read(1) == '[':
            return {'A': 'UP', 'B': 'DOWN', 'C': 'RIGHT', 'D': 'LEFT'}.get(sys.stdin.read(1), '')
        return ''
    return ch


def main():
    rclpy.init()
    node = ServoKeyboard()

    exe = SingleThreadedExecutor()
    exe.add_node(node)

    fd  = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setraw(fd)

    period = 1.0 / RATE_HZ
    try:
        print('MoveIt Servo keyboard — q to quit')
        print('Arrow/./; → linear | a/d/f/v/x/c → rotation | 1–7 → joint')
        print('j/t → mode | w/e → frame | r → reverse | s → stop | release key → coast ends\r')
        while rclpy.ok():
            # Drain available keys without blocking (holding keys may not repeat).
            while True:
                r, _, _ = select.select([sys.stdin], [], [], 0)
                if not r:
                    break
                key = read_key()
                if key == 'q':
                    return
                if key:
                    node.on_key(key)
            # Drive the 50 Hz timer in this thread (background spin can miss timers).
            exe.spin_once(timeout_sec=period)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
