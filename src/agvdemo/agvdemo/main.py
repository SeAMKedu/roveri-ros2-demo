"""
Simple package for receiving the position of the GNSS receiver.
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class PositionSubscriber(Node):
    """Subscribe to the topic of the position of the GNSS receiver."""

    def __init__(self):
        super().__init__("possub")
        self.sub = self.create_subscription(String, "gnsspos", self.callback, 10)
        self.sub
    
    def callback(self, msg: String):
        self.get_logger().info(msg.data)


def main(args=None):
    """Run the node."""
    rclpy.init(args=args)
    possub = PositionSubscriber()
    try:
        rclpy.spin(node=possub)
    except (KeyboardInterrupt, ExternalShutdownException):
        print("Shutting down the node...")
    finally:
        possub.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
