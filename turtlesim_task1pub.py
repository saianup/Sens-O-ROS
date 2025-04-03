import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtleVelocityPublisher(Node):
    def __init__(self):
        super().__init__('turtle_velocity_publisher')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.5, self.publish_velocity)
        self.get_logger().info('Turtle Velocity Publisher Node Started')

    def publish_velocity(self):
        msg = Twist()
        msg.linear.x = 3.0  # Move forward
        msg.angular.z = 1.0  # Rotate
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing Velocity: linear.x={msg.linear.x}, angular.z={msg.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    node = TurtleVelocityPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
