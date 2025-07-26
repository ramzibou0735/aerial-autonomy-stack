import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import argparse

class MinimalTalker(Node):
    def __init__(self, drone_id):
        # Construct the node name with the ID
        super().__init__(f'minimal_talker_{drone_id}')
        
        # Construct the topic name with the ID
        topic_name = f'test_topic_{drone_id}'
        
        self.publisher_ = self.create_publisher(String, topic_name, 10)
        self.timer = self.create_timer(1.0, self.timer_callback) # 1 Hz
        self.i = 0
        self.drone_id = drone_id

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello from Drone {self.drone_id}: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing to {self.publisher_.topic_name}: "{msg.data}"')
        self.i += 1

def main(args=None):
    # Standard ROS2 and argparse setup
    rclpy.init(args=args)
    
    parser = argparse.ArgumentParser(description='Simple ROS2 Talker with dynamic ID.')
    parser.add_argument('--drone-id', type=int, default=1, help='ID of the drone.')
    
    # ROS2 uses --ros-args, so we parse known args first
    cli_args, _ = parser.parse_known_args()
    
    minimal_talker = MinimalTalker(drone_id=cli_args.drone_id)
    
    rclpy.spin(minimal_talker)
    
    minimal_talker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()