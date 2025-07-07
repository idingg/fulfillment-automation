import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TestSubscriber(Node):
    def __init__(self):
        super().__init__("test_subscriber")
        self.subscription = self.create_subscription(
            String, "/job_selection", self.listener_callback, 10
        )
        self.subscription 

    def listener_callback(self, msg):
        self.get_logger().info(f"Received message: {msg.data}")


def main(args=None):
    rclpy.init(args=args)  
    test_subscriber = TestSubscriber()  
    try:
        rclpy.spin(test_subscriber)  
    except KeyboardInterrupt:
        pass  
    finally:
        test_subscriber.destroy_node()  
        rclpy.shutdown()


if __name__ == "__main__":
    main()
