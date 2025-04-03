import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math

class TurtleFollower(Node):
    def __init__(self):
        super().__init__('turtle_follower')
        
        self.subscription1 = self.create_subscription(Pose, '/turtle1/pose', self.turtle1_pose_callback, 10)
        self.subscription2 = self.create_subscription(Pose, '/turtle2/pose', self.turtle2_pose_callback, 10)
        
        self.publisher_ = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
        
        self.turtle1_pose = None
        self.turtle2_pose = None

        self.get_logger().info('Turtle2 will follow Turtle1!')

    def turtle1_pose_callback(self, msg):
        self.turtle1_pose = msg
        self.move_turtle2()

    def turtle2_pose_callback(self, msg):
        self.turtle2_pose = msg

    def move_turtle2(self):
        if self.turtle1_pose is None or self.turtle2_pose is None:
            return

        
        dx = self.turtle1_pose.x - self.turtle2_pose.x
        dy = self.turtle1_pose.y - self.turtle2_pose.y
        distance = math.sqrt(dx**2 + dy**2)
        angle_to_target = math.atan2(dy, dx)

        
        angle_diff = angle_to_target - self.turtle2_pose.theta
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))  
        twist_msg = Twist()


        if distance > 0.5:
            twist_msg.linear.x = min(1.2 * distance, 1.5)  
        else:
            twist_msg.linear.x = 0.0

      
        twist_msg.angular.z = 2.0 * angle_diff  
        twist_msg.angular.z = max(min(twist_msg.angular.z, 2.0), -2.0)  

        self.publisher_.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
