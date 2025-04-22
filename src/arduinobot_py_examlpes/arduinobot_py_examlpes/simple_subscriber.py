import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        self.sub_ = self.create_subscription(String, 'chatter', self.msgCallback, 10)

    def msgCallback(self, msg):
        self.get_logger().info(f"Received message: {msg.data}")

def main():
    rclpy.init() ## initialize the ROS client library
    simple_subscriber = SimpleSubscriber() ## create a SimpleSubscriber object
    rclpy.spin(simple_subscriber) ## start the ROS client library
    simple_subscriber.destroy_node() ## destroy the SimpleSubscriber object
    rclpy.shutdown() ## shutdown the ROS client library


if __name__ == '__main__':
    main()

