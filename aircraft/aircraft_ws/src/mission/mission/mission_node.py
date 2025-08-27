import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup

import argparse
import threading

from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import VfrHud
from vision_msgs.msg import Detection2DArray
from px4_msgs.msg import VehicleGlobalPosition, AirspeedValidated

from ground_system_msgs.msg import SwarmObs
from state_sharing.msg import SharedState 

class MissionNode(Node):
    def __init__(self, conops):
        super().__init__('mission_node')
        self.conops = conops
        self.get_logger().info(f"Missioning with CONOPS: {self.conops}")

        self.data_lock = threading.Lock()
        # MAVROS data
        self.lat = None
        self.lon = None
        self.alt_msl = None
        self.heading = None
        self.airspeed = None
        # Perception data
        self.yolo_detections = None
        # self.ground_tracks = None
        # State sharing
        self.active_state_sharing_subs = {}
        self.drone_states = {}

        # Create a reentrant callback groups to allow callbacks to run in parallel
        self.subscriber_callback_group = ReentrantCallbackGroup()
        self.timer_callback_group = ReentrantCallbackGroup()

        # Create a QoS profile for the subscribers
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        # PX4 subscribers
        self.create_subscription( # 100Hz
            VehicleGlobalPosition, 'fmu/out/vehicle_global_position', self.px4_global_position_callback,
            self.qos_profile, callback_group=self.subscriber_callback_group)
        self.create_subscription( # 10Hz
            AirspeedValidated, '/fmu/out/airspeed_validated', self.airspeed_validated_callback,
            self.qos_profile, callback_group=self.subscriber_callback_group)
        # MAVROS subscribers
        self.create_subscription( # 4Hz
            NavSatFix, '/mavros/global_position/global', self.mavros_global_position_callback,
            self.qos_profile, callback_group=self.subscriber_callback_group)
        self.create_subscription( # 4Hz
            VfrHud, '/mavros/vfr_hud', self.vfr_hud_callback,
            self.qos_profile, callback_group=self.subscriber_callback_group)
        # Perception subscribers
        self.create_subscription( # 15Hz
            Detection2DArray, '/detections', self.yolo_detections_callback,
            self.qos_profile, callback_group=self.subscriber_callback_group)
        # self.create_subscription( # 1Hz
        #     SwarmObs, '/tracks', self.ground_tracks_callback,
        #     self.qos_profile, callback_group=self.subscriber_callback_group)

        # Timed callbacks
        self.discover_drones_timer = self.create_timer(
            5.0, # 0.2Hz
            self.discover_drones_callback,
            callback_group=self.timer_callback_group
        )
        self.printout_timer = self.create_timer(
            3.0, # 0.33Hz
            self.printout_callback, 
            callback_group=self.timer_callback_group
        )
        self.conops_timer = self.create_timer(
            1.0, # 1Hz
            self.conops_callback, 
            callback_group=self.timer_callback_group
        )

    def px4_global_position_callback(self, msg): # Mutally exclusive with mavros_global_position_callback
        with self.data_lock:
            self.lat = msg.lat
            self.lon = msg.lon
            self.alt_msl = msg.alt
    
    def airspeed_validated_callback(self, msg): # Mutally exclusive with vfr_hud_callback
        with self.data_lock:
            self.airspeed = msg.true_airspeed_m_s
    
    def mavros_global_position_callback(self, msg):  # Mutally exclusive with px4_global_position_callback
        with self.data_lock:
            self.lat = msg.latitude
            self.lon = msg.longitude

    def vfr_hud_callback(self, msg): # Mutally exclusive with airspeed_validated_callback
        with self.data_lock:
            self.alt_msl = msg.altitude
            self.heading = msg.heading
            self.airspeed = msg.airspeed

    def yolo_detections_callback(self, msg):
        with self.data_lock:
            self.yolo_detections = msg

    def discover_drones_callback(self):
        topic_prefix = '/state_sharing_drone_'        
        current_topics = self.get_topic_names_and_types()
        for topic_name, msg_types in current_topics:
            if topic_name.startswith(topic_prefix) and topic_name not in self.active_state_sharing_subs:
                if 'state_sharing/msg/SharedState' in msg_types:
                    self.get_logger().info(f"Discovered new drone: subscribing to {topic_name}")
                    sub = self.create_subscription( # 1Hz
                        SharedState,
                        topic_name,
                        self.state_sharing_callback,
                        self.qos_profile,
                        callback_group=self.subscriber_callback_group
                    )
                    self.active_state_sharing_subs[topic_name] = sub # Store the subscriber

    def state_sharing_callback(self, msg):
        # A single callback for all drone state topics
        with self.data_lock: 
            self.drone_states[msg.drone_id] = msg

    # def ground_tracks_callback(self, msg):
    #     with self.data_lock:
    #         self.ground_tracks = msg

    def printout_callback(self):
        with self.data_lock: # Copy with lock
            lat = self.lat
            lon = self.lon
            alt_msl = self.alt_msl
            yolo_detections = self.yolo_detections
            states_copy = self.drone_states.copy()
            # ground_tracks = self.ground_tracks
        now_seconds = self.get_clock().now().nanoseconds / 1e9
        output = f"\nCurrent node time: {now_seconds:.2f} seconds\n"
        lat_str = f"{lat:.5f}" if lat is not None else "N/A"
        lon_str = f"{lon:.5f}" if lon is not None else "N/A"
        alt_str = f"{alt_msl:.2f}" if alt_msl is not None else "N/A"
        output += f"Global Position:\n  lat: {lat_str} lon: {lon_str} alt: {alt_str} (msl)\n"
        #
        if yolo_detections and yolo_detections.detections:
            output += "YOLO Detections:\n"
            for detection in yolo_detections.detections:
                for result in detection.results:
                    output += f"  Label: {result.hypothesis.class_id} - conf: {result.hypothesis.score:.2f}\n"
        else:
            output += "YOLO Detections: [No data]\n"
        #
        if not states_copy:
            output += "State Sharing: [No data]\n"
        else:
            output += "State Sharing:\n"
            for drone_id in sorted(states_copy.keys()):
                state = states_copy[drone_id]
                output += f"  Id {drone_id}, lat: {state.latitude_deg:.5f} lon: {state.longitude_deg:.5f} alt: {state.altitude_m:.2f} (px4: msl, ap: ell.)\n"
        #
        # if ground_tracks and ground_tracks.tracks:
        #     output += "Ground Tracks:\n"
        #     for track in ground_tracks.tracks:
        #         output += f"  Id {track.id}, lat: {track.latitude_deg:.5f} lon: {track.longitude_deg:.5f} alt (rel): {track.altitude_m:.2f}\n"
        # else:
        #     output += "Ground Tracks: [No data]\n"
        #
        self.get_logger().info(output)

    def conops_callback(self):
        if self.conops == 'plan_A':
            self.get_logger().info("Plan A is classified")
        elif self.conops == 'plan_B':
            self.get_logger().info("There is no Plan B")
        elif self.conops == 'yalla':
            self.get_logger().info("Yalla")
        else:
            self.get_logger().warn(f"Unknown CONOPS: {self.conops}")

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
