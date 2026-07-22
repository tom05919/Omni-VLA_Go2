import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import Twist
from PIL import Image as PILImage
from sensor_msgs.msg import Image as RosImage

SIM_TOPICS = {
    "image": "/unitree_go2_0/front_cam/color_image",
    "cmd_vel": "/unitree_go2_0/cmd_vel",
}
REAL_TOPICS = {
    "image": "/camera/image_raw",
    "cmd_vel": "/cmd_vel",
}


def clip_angle(theta):
    """Wrap angle to [-pi, pi]."""
    return np.arctan2(np.sin(theta), np.cos(theta))


def _image_qos() -> QoSProfile:
    # Match Go2 driver camera publisher (BEST_EFFORT).
    return QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=5,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
    )


class IsaacSimPublisher(Node):
    def __init__(
        self,
        sim: bool = False,
        image_topic: str | None = None,
        cmd_vel_topic: str | None = None,
        node_name: str = "cmd_vel_publisher",
    ):
        topics = SIM_TOPICS if sim else REAL_TOPICS
        image_topic = image_topic or topics["image"]
        self._sim = sim
        self.cmd_vel_topic = cmd_vel_topic or topics["cmd_vel"]
        super().__init__(node_name)

        self.pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self._last_twist = Twist()
        self._cmd_timer = self.create_timer(0.1, self._republish_cmd_vel)
        self.latest_image = None
        self.img_sub = self.create_subscription(
            RosImage, image_topic, self._image_callback, _image_qos()
        )
        self._spin_stop = threading.Event()
        self._spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
        self._spin_thread.start()

    def _spin_loop(self):
        while not self._spin_stop.is_set() and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def destroy_node(self):
        self._spin_stop.set()
        if self._spin_thread.is_alive():
            self._spin_thread.join(timeout=1.0)
        super().destroy_node()

    def _republish_cmd_vel(self):
        self.pub.publish(self._last_twist)

    def _image_callback(self, msg):
        """Convert sensor_msgs/Image into an (H, W, 3) uint8 RGB array."""
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        arr = arr.reshape(msg.height, msg.step)
        arr = arr[:, : msg.width * 3].reshape(msg.height, msg.width, 3)
        if msg.encoding.lower() in ("bgr8", "bgra8"):
            arr = arr[..., :3][..., ::-1]
        self.latest_image = arr

    def get_latest_image_pil(self, timeout_sec=5.0, fresh=True):
        """Wait for the background spin thread to deliver a camera frame."""
        if fresh:
            self.latest_image = None
        start = time.time()
        while self.latest_image is None and (time.time() - start) < timeout_sec:
            time.sleep(0.05)
        if self.latest_image is None:
            return None
        return PILImage.fromarray(self.latest_image.copy(), mode="RGB")

    def stop(self):
        if not rclpy.ok():
            return
        self._last_twist = Twist()
        self.pub.publish(self._last_twist)

    def publish_velocity(self, linear, angular, isaac_scale=None):
        """Publish velocity commands; 10 Hz timer keeps the stream alive for Go2/twist_mux."""
        if isaac_scale is None:
            isaac_scale = 1.0 / 0.6 if self._sim else 1.0
        self._last_twist = Twist()
        self._last_twist.linear.x = float(linear * isaac_scale)
        self._last_twist.angular.z = float(angular * isaac_scale)
        self.pub.publish(self._last_twist)
        print(
            f"cmd_vel -> {self.cmd_vel_topic}: "
            f"vx={self._last_twist.linear.x:.3f} wz={self._last_twist.angular.z:.3f}"
        )
