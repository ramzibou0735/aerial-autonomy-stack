import rclpy
from rclpy.node import Node
import argparse
import multiprocessing as mp
from rclpy.executors import SingleThreadedExecutor

# Import your custom message
from ground_system_msgs.msg import SwarmObs

# def subscriber_process(q, sub_domain_id):
#     """Process that subscribes to the topic on the source domain."""
#     try:
#         context = rclpy.Context()
#         rclpy.init(context=context, args=['--ros-domain-id', str(sub_domain_id)])
#         node = rclpy.create_node('sub_node', context=context)
        
#         def listener_callback(msg):
#             # For multiprocessing, we pass serialized data through the queue
#             q.put(msg)

#         sub = node.create_subscription(SwarmObs, '/tracks', listener_callback, 10)
#         rclpy.spin(node, context=context)

#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown(context=context)

def subscriber_process(q, sub_domain_id):
    """Process that subscribes to the topic on the source domain."""
    context = rclpy.Context()
    rclpy.init(context=context, args=['--ros-domain-id', str(sub_domain_id)])
    
    node = rclpy.create_node('sub_node', context=context)
    
    def listener_callback(msg):
        q.put(msg)

    sub = node.create_subscription(SwarmObs, '/tracks', listener_callback, 10)
    
    # --- This is the corrected part ---
    # Create an executor and add the node to it
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    
    try:
        # Spin the executor instead of rclpy.spin()
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)

def publisher_process(q, pub_domain_id):
    """Process that publishes the topic on the destination domain."""
    try:
        context = rclpy.Context()
        rclpy.init(context=context, args=['--ros-domain-id', str(pub_domain_id)])
        node = rclpy.create_node('pub_node', context=context)
        publisher = node.create_publisher(SwarmObs, '/tracks', 10)
        
        while rclpy.ok(context=context):
            try:
                msg = q.get(timeout=0.1)
                publisher.publish(msg)
            except mp.queues.Empty:
                pass
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown(context=context)


def main(args=None):
    parser = argparse.ArgumentParser(description='ROS 2 Multi-Process Relay')
    parser.add_argument('--sub-domain', type=int, required=True, help="Domain ID to subscribe FROM.")
    parser.add_argument('--pub-domain', type=int, required=True, help="Domain ID to publish TO.")
    cli_args = parser.parse_args()

    # Use a multiprocessing queue for safe inter-process communication
    msg_queue = mp.Queue()

    # Create and start the subscriber and publisher processes
    sub_proc = mp.Process(target=subscriber_process, args=(msg_queue, cli_args.sub_domain))
    pub_proc = mp.Process(target=publisher_process, args=(msg_queue, cli_args.pub_domain))
    
    sub_proc.start()
    pub_proc.start()
    
    print(f"Relaying from domain {cli_args.sub_domain} to {cli_args.pub_domain}...")
    
    try:
        sub_proc.join()
        pub_proc.join()
    except KeyboardInterrupt:
        print("Shutting down processes...")
    finally:
        sub_proc.terminate()
        pub_proc.terminate()

if __name__ == '__main__':
    main()