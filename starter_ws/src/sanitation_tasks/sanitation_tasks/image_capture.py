import time

from PIL import Image
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as ImageMessage


class ImageCapture(Node):
    def __init__(self):
        super().__init__("image_capture")
        self.declare_parameter("output_path", "/tmp/overview.png")
        self.declare_parameter("timeout_sec", 30.0)
        self.message = None
        self.create_subscription(ImageMessage, "/world_overview/image", self._image, 5)

    def _image(self, message):
        self.message = message

    def run(self):
        started = time.monotonic()
        while rclpy.ok() and self.message is None and time.monotonic() - started < self.get_parameter("timeout_sec").value:
            rclpy.spin_once(self, timeout_sec=0.2)
        if self.message is None:
            return False
        message = self.message
        modes = {"rgb8": "RGB", "bgr8": "RGB", "rgba8": "RGBA", "bgra8": "RGBA", "mono8": "L"}
        if message.encoding not in modes:
            raise RuntimeError(f"unsupported overview encoding: {message.encoding}")
        channels = len(modes[message.encoding])
        raw = bytes(message.data)
        rows = [raw[index * message.step:index * message.step + message.width * channels] for index in range(message.height)]
        image = Image.frombytes(modes[message.encoding], (message.width, message.height), b"".join(rows))
        if message.encoding.startswith("bgr"):
            red, green, blue = image.split()[:3]
            image = Image.merge(image.mode, (blue, green, red) + image.split()[3:])
        image.save(self.get_parameter("output_path").value)
        return True


def main(args=None):
    rclpy.init(args=args)
    node = ImageCapture()
    try:
        success = node.run()
    finally:
        node.destroy_node(); rclpy.shutdown()
    if not success:
        raise SystemExit(2)
