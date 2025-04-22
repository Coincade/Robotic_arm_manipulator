import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher') ## initialize the node
        self.pub_ = self.create_publisher(String, 'chatter', 10) ## create a publisher
        self.counter_ = 0 ## initialize the counter
        self.frequency_ = 1.0 ## initialize the frequency
        self.get_logger().info(f"Publishing at {self.frequency_} Hz") ## log the frequency
        self.timer_ = self.create_timer(self.frequency_, self.timerCallback) ## create a timer

    def timerCallback(self):
        msg = String()
        msg.data = f"Hello, ROS 2! {self.counter_}"
        self.pub_.publish(msg)
        self.counter_ += 1

def main():
    rclpy.init() ## initialize the ROS client library
    simple_publisher = SimplePublisher() ## create a SimplePublisher object
    rclpy.spin(simple_publisher) ## start the ROS client library
    simple_publisher.destroy_node() ## destroy the SimplePublisher object
    rclpy.shutdown() ## shutdown the ROS client library




if __name__ == '__main__':
    main()

