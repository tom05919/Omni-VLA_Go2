import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import numpy as np


def clip_angle(theta):
    """Wrap angle to [-pi, pi]."""
    return np.arctan2(np.sin(theta), np.cos(theta))


def limit_velocity(linear, angular, maxv=0.3, maxw=0.3):
    """Same magnitude-limiting scheme as run_omnivla_edge.py."""
    if abs(linear) <= maxv:
        if abs(angular) <= maxw:
            return linear, angular
        rd = linear / angular
        return maxw * np.sign(linear) * abs(rd), maxw * np.sign(angular)
    if abs(angular) <= 0.001:
        return maxv * np.sign(linear), 0.0
    rd = linear / angular
    if abs(rd) >= maxv / maxw:
        return maxv * np.sign(linear), maxv * np.sign(angular) / abs(rd)
    return maxw * np.sign(linear) * abs(rd), maxw * np.sign(angular)


def segment_to_twist(dx, dy, hx, hy, dt=1.0 / 3.0):
    """Convert one waypoint segment (meters, robot frame) -> (vx, wz)."""
    EPS = 1e-8
    if abs(dx) < EPS and abs(dy) < EPS:
        vx, wz = 0.0, clip_angle(np.arctan2(hy, hx)) / dt
    elif abs(dx) < EPS:
        vx, wz = 0.0, np.sign(dy) * np.pi / (2 * dt)
    else:
        vx = dx / dt
        wz = np.arctan2(dy, dx) / dt
    vx = np.clip(vx, 0, 0.5)
    wz = np.clip(wz, -1.0, 1.0)
    return limit_velocity(vx, wz)


class IsaacSimPublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.pub = self.create_publisher(Twist, '/unitree_go2_0/cmd_vel', 10)

    def stop(self):
        msg = Twist()  # all zeros
        for _ in range(5):  # send a few times for reliability
            self.pub.publish(msg)
            time.sleep(0.05)

    def execute_waypoints(self, waypoints,
                          spacing=0.1,            # metric_waypoint_spacing
                          dt=1.0 / 0.5,           # seconds per waypoint segment
                          isaac_scale=1.0 / 0.15,  # map ~0.3 m/s -> ~1.0 policy cmd
                          publish_hz=20):
        """
        waypoints: np.ndarray [8, 4] (cumulative x,y normalized + cos,sin),
        i.e. predicted_actions[0].cpu().numpy()
        """
        wp = np.asarray(waypoints, dtype=np.float64).copy()
        wp[:, :2] *= spacing  # normalized -> meters

        prev = np.array([0.0, 0.0])  # robot starts at origin of its own frame
        for i in range(wp.shape[0]):
            dx = wp[i, 0] - prev[0]
            dy = wp[i, 1] - prev[1]
            hx, hy = wp[i, 2], wp[i, 3]
            prev = wp[i, :2]

            vx, wz = segment_to_twist(dx, dy, hx, hy, dt)

            # scale into the Isaac Go2 policy's command range
            vx *= isaac_scale
            wz *= isaac_scale

            self._hold_cmd(vx, 0.0, wz, duration=dt, hz=publish_hz)

        self.stop()

    def _hold_cmd(self, vx, vy, wz, duration, hz=20):
        """Publish a Twist repeatedly for `duration` seconds (sim has no watchdog)."""
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(wz)
        n = max(1, int(duration * hz))
        for _ in range(n):
            self.pub.publish(msg)
            time.sleep(1.0 / hz)
