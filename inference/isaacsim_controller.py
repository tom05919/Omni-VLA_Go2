import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image as RosImage
from PIL import Image as PILImage
import time
import numpy as np


def clip_angle(theta):
    """Wrap angle to [-pi, pi]."""
    return np.arctan2(np.sin(theta), np.cos(theta))


class IsaacSimPublisher(Node):
    def __init__(self, image_topic='/unitree_go2_0/front_cam/color_image'):
        super().__init__('cmd_vel_publisher')
        self.pub = self.create_publisher(Twist, '/unitree_go2_0/cmd_vel', 10)

        # Camera intake: store the most recent frame from the Go2.
        self.latest_image = None
        self.img_sub = self.create_subscription(
            RosImage, image_topic, self._image_callback, 10)

    def _image_callback(self, msg):
        """Convert an rgb8 sensor_msgs/Image into an (H, W, 3) uint8 array."""
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        # msg.step is the row stride in bytes; slice off any row padding.
        arr = arr.reshape(msg.height, msg.step)
        arr = arr[:, : msg.width * 3].reshape(msg.height, msg.width, 3)
        self.latest_image = arr

    def get_latest_image_pil(self, timeout_sec=5.0, fresh=True):
        """Spin until a (fresh) camera frame is available; return it as a PIL RGB image.

        fresh=True drops any previously buffered frame and waits for a new one,
        so each inference tick uses an image captured after the last motion.
        Returns None if no frame arrives within timeout_sec.
        """
        if fresh:
            self.latest_image = None
        start = time.time()
        while self.latest_image is None and (time.time() - start) < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.latest_image is None:
            return None
        return PILImage.fromarray(self.latest_image, mode='RGB')

    def stop(self):
        msg = Twist()  # all zeros
        for _ in range(5):  # send a few times for reliability
            self.pub.publish(msg)
            time.sleep(0.05)

    def publish_velocity(self, linear, angular, isaac_scale=1.0 / 0.3):
        """Publish a single velocity command (Twist), OmniVLA/ViNT-style.

        The inference loop's PD controller produces one (linear, angular)
        command per model output. We publish it once and leave it active until
        the next model output overwrites it (the Isaac bridge has no cmd_vel
        watchdog, so the last command persists). This matches the paper, where
        the active velocity command is updated whenever a new output arrives.

        `isaac_scale` maps the model's physical velocities (capped at ~0.3 m/s
        and rad/s by the PD limiter) into the Go2 policy's command range, where
        keyboard teleop uses magnitudes around 1.0.
        """
        msg = Twist()
        msg.linear.x = float(linear * isaac_scale)
        msg.angular.z = float(angular * isaac_scale)
        self.pub.publish(msg)
