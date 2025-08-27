import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

import argparse

class MissionNode(Node):
    def __init__(self, conops):
        super().__init__('mission_node')
        print("TODO")

def main(args=None):
    parser = argparse.ArgumentParser(description="Mission Node.")
    parser.add_argument(
        '--conops',
        type=str,
        choices=['plan_A', 'plan_B', 'yalla'],
        default='yalla',
        help="Specify the concept of operations."
    )
        
    cli_args, ros_args = parser.parse_known_args()
    
    rclpy.init(args=ros_args)
    mission_node = MissionNode(conops=cli_args.conops)

    executor = MultiThreadedExecutor() # Or set MultiThreadedExecutor(num_threads=4)
    executor.add_node(mission_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        mission_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
