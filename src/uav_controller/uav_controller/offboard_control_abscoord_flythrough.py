#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# --- Import message types ---
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

# --- Import TF2 related modules ---
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import math
import copy

class AbsoluteCoordMission(Node):
    """
    Node for offboard control using fly-through waypoints.
    - The drone flies through a series of waypoints without stopping.
    - It switches to the next waypoint upon entering an "acceptance radius" of the current one.
    - Gimbal points to the corresponding GPS target for each waypoint.
    """

    def __init__(self):
        super().__init__('absolute_coord_mission_node')
        self.set_parameters([Parameter('use_sim_time', value=True)])

        # --- Publishers ---
        self.offboard_control_mode_publisher = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", 10)
        self.trajectory_setpoint_publisher = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", 10)
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)
        self.state_publisher = self.create_publisher(String, "/drone/state", 10)

        # --- Subscriber ---
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, "/fmu/out/vehicle_local_position", self.local_position_callback, qos_profile
        )

        # --- TF2 Listener ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Mission Variables ---
        self.state = "INIT"
        self.current_map_pose = None
        self.current_local_pos = None
        self.target_pose_map = PoseStamped()
        self.target_pose_map.header.frame_id = 'map'
        self.takeoff_target_local = None
        
        self.handshake_counter = 0
        self.handshake_duration = 15
        self.hover_counter = 0
        self.hover_duration = 50 
        self.drone_frame_id = "x500_gimbal_0/base_link" 
        self.next_state_after_hover = ""

        # --- Drone Waypoint Mission (World Coordinates - map frame) ---
        self.waypoints = [
            [-94.4088, 68.4708, 13.8531],
            [-75.4421, 74.9961, 33.2347],
            [-65.0308, 80.1275, 18.4990],
            [-82.7931, 113.4203, 13.8079],
            [-97.9238, 105.2799, 18.5504],
            [-109.1330, 100.3533, 33.1363]
        ]
        
        # --- Gimbal Target Mission (GPS Coordinates) ---
        self.gimbal_targets = [
            [37.413118, -122.001327, 42.43],
            [37.413165, -122.001102, 61.81],
            [37.413213, -122.000983, 50.65],
            [37.413509, -122.001180, 45.50],
            [37.413440, -122.001355, 50.00],
            [37.413397, -122.001478, 65.00]
        ]

        self.final_destination = [-62.9630, 99.0915, 0.1349]
        self.waypoint_index = 0
        
        # <<< CHANGE START >>>
        # --- Fly-through configuration ---
        self.acceptance_radius = 5.0  # (meters) Start moving to the next waypoint when within this distance.
        self.cruise_altitude = max(wp[2] for wp in self.waypoints) + 10.0 # Keep a safe margin above all waypoints
        # <<< CHANGE END >>>

        self.get_logger().info("Fly-through Mission Controller is running.")
        self.get_logger().info(f"Cruise altitude set to: {self.cruise_altitude:.2f}m")
        self.get_logger().info(f"Waypoint acceptance radius: {self.acceptance_radius:.2f}m")
        self.get_logger().info("Waiting for TF and Local Position data to start...")

    def local_position_callback(self, msg: VehicleLocalPosition):
        self.current_local_pos = msg
        self.run_state_machine()

    def update_current_map_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', self.drone_frame_id, rclpy.time.Time())
            self.current_map_pose = PoseStamped()
            self.current_map_pose.pose.position.x = trans.transform.translation.x
            self.current_map_pose.pose.position.y = trans.transform.translation.y
            self.current_map_pose.pose.position.z = trans.transform.translation.z
            return True
        except TransformException as e:
            if self.state != "INIT":
                self.get_logger().warn(f"Could not get current drone pose in map frame: {e}", throttle_duration_sec=1.0)
            return False

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_map_based_setpoint(self):
        if self.current_map_pose is None or self.current_local_pos is None:
            return

        delta_map_x = self.target_pose_map.pose.position.x - self.current_map_pose.pose.position.x
        delta_map_y = self.target_pose_map.pose.position.y - self.current_map_pose.pose.position.y
        delta_map_z = self.target_pose_map.pose.position.z - self.current_map_pose.pose.position.z
        
        delta_ned_x, delta_ned_y, delta_ned_z = delta_map_y, delta_map_x, -delta_map_z

        target_ned_x = self.current_local_pos.x + delta_ned_x
        target_ned_y = self.current_local_pos.y + delta_ned_y
        target_ned_z = self.current_local_pos.z + delta_ned_z

        msg = TrajectorySetpoint()
        msg.position = [float(target_ned_x), float(target_ned_y), float(target_ned_z)]
        msg.yaw = math.nan # Let PX4 control yaw towards the direction of travel
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_vehicle_command(self, command, **kwargs):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = kwargs.get("param1", 0.0)
        msg.param2 = kwargs.get("param2", 0.0)
        msg.param3 = kwargs.get("param3", 0.0)
        msg.param4 = kwargs.get("param4", 0.0)
        msg.param5 = kwargs.get("param5", 0.0)
        msg.param6 = kwargs.get("param6", 0.0)
        msg.param7 = kwargs.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)
        
    def point_gimbal_at_target(self, target_index):
        if target_index < len(self.gimbal_targets):
            target = self.gimbal_targets[target_index]
            lat, lon, alt = target[0], target[1], target[2]
            
            self.get_logger().info(f"Commanding gimbal to point at Marker #{target_index} (Lat: {lat}, Lon: {lon}, Alt: {alt})")
            
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_MOUNT_CONTROL,
                param4=float(alt), param5=float(lat), param6=float(lon), param7=4.0
            )
        else:
            self.get_logger().warn(f"Gimbal target index {target_index} is out of bounds.")
    
    # <<< CHANGE START >>>
    def check_waypoint_proximity(self, radius):
        """Check if the drone is within a given radius of the target map position."""
        if self.current_map_pose is None: return False
        
        pos = self.current_map_pose.pose.position
        target_pos = self.target_pose_map.pose.position
        
        distance = math.sqrt((pos.x - target_pos.x)**2 + (pos.y - target_pos.y)**2 + (pos.z - target_pos.z)**2)
        return distance < radius
    # <<< CHANGE END >>>

    def run_state_machine(self):
        if not self.update_current_map_pose() or self.current_local_pos is None:
            return

        state_msg = String()
        state_msg.data = self.state
        self.state_publisher.publish(state_msg)

        if self.state not in ["LAND", "MISSION_COMPLETE"]:
             self.publish_offboard_control_mode()

        if self.state == "INIT":
            self.get_logger().info("TF and Local Position received. Starting handshake.")
            self.state = "HANDSHAKE"

        elif self.state == "HANDSHAKE":
            sp_msg = TrajectorySetpoint()
            sp_msg.position = [self.current_local_pos.x, self.current_local_pos.y, self.current_local_pos.z]
            self.trajectory_setpoint_publisher.publish(sp_msg)
            
            self.handshake_counter += 1
            if self.handshake_counter > self.handshake_duration:
                self.get_logger().info("Handshake complete. Switching to Offboard mode and Arming.")
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
                self.state = "ARMING"

        elif self.state == "ARMING":
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            self.get_logger().info("Arm command sent.")
            takeoff_altitude = 10.0
            self.takeoff_target_local = [self.current_local_pos.x, self.current_local_pos.y, self.current_local_pos.z - takeoff_altitude]
            self.state = "TAKEOFF"
        
        elif self.state == "TAKEOFF":
            sp_msg = TrajectorySetpoint()
            sp_msg.position = [float(p) for p in self.takeoff_target_local]
            self.trajectory_setpoint_publisher.publish(sp_msg)
            
            if abs(self.current_local_pos.z - self.takeoff_target_local[2]) < 1.0:
                self.get_logger().info("Takeoff complete. Hovering before mission start.")
                self.target_pose_map = copy.deepcopy(self.current_map_pose) # Set target to current for stable hover
                self.state = "HOVER"
                self.next_state_after_hover = "ASCEND_TO_CRUISE" # <<< CHANGE
                self.hover_counter = 0

        elif self.state == "HOVER":
            self.publish_map_based_setpoint()
            self.hover_counter += 1
            if self.hover_counter > self.hover_duration:
                self.get_logger().info(f"Hover complete. Proceeding to: {self.next_state_after_hover}")
                self.state = self.next_state_after_hover
                self.hover_counter = 0
        
        # <<< CHANGE START >>>
        elif self.state == "ASCEND_TO_CRUISE":
            # Ascend to a safe cruising altitude at the current XY location before starting the mission.
            self.get_logger().info(f"Ascending to cruise altitude: {self.cruise_altitude:.2f}m")
            self.target_pose_map.pose.position.z = self.cruise_altitude
            self.state = "EXECUTING_MOVE"
            self.next_state_after_hover = "START_FLYING_MISSION"

        elif self.state == "START_FLYING_MISSION":
            # Set the first waypoint as the initial target and switch to the main mission state.
            self.get_logger().info("Starting fly-through mission.")
            wp = self.waypoints[self.waypoint_index]
            self.target_pose_map.pose.position.x = wp[0]
            self.target_pose_map.pose.position.y = wp[1]
            self.target_pose_map.pose.position.z = wp[2]
            self.point_gimbal_at_target(self.waypoint_index)
            self.get_logger().info(f"--> Moving to Waypoint #{self.waypoint_index} at ({wp[0]:.2f}, {wp[1]:.2f}, {wp[2]:.2f})")
            self.state = "FLYING_MISSION"

        elif self.state == "FLYING_MISSION":
            # This is the main state for the fly-through logic.
            self.publish_map_based_setpoint()

            # Check if we are close enough to the current waypoint to switch to the next one.
            if self.check_waypoint_proximity(self.acceptance_radius):
                self.get_logger().info(f"Reached proximity of Waypoint #{self.waypoint_index}.")
                self.waypoint_index += 1

                if self.waypoint_index < len(self.waypoints):
                    # If there are more waypoints, set the next one as the target.
                    wp = self.waypoints[self.waypoint_index]
                    self.target_pose_map.pose.position.x = wp[0]
                    self.target_pose_map.pose.position.y = wp[1]
                    self.target_pose_map.pose.position.z = wp[2]
                    # Point the gimbal to the new target.
                    self.point_gimbal_at_target(self.waypoint_index)
                    self.get_logger().info(f"--> Moving to Waypoint #{self.waypoint_index} at ({wp[0]:.2f}, {wp[1]:.2f}, {wp[2]:.2f})")
                else:
                    # All waypoints have been visited.
                    self.get_logger().info("All waypoints visited. Moving to final destination.")
                    self.state = "MOVE_TO_FINAL_DESTINATION"
        
        elif self.state == "MOVE_TO_FINAL_DESTINATION":
            dest = self.final_destination
            self.target_pose_map.pose.position.x = dest[0]
            self.target_pose_map.pose.position.y = dest[1]
            # Approach the final destination at a safe altitude first.
            self.target_pose_map.pose.position.z = self.cruise_altitude
            self.state = "EXECUTING_MOVE"
            # After arriving above the landing spot, hover, then land.
            self.next_state_after_hover = "LAND"

        elif self.state == "EXECUTING_MOVE":
            # This state is now only used for single, precise moves where arrival is required.
            # (e.g., ascending to cruise, moving to final destination)
            self.publish_map_based_setpoint()
            # Use a tight tolerance for arrival at these key points.
            if self.check_waypoint_proximity(1.0): 
                self.get_logger().info("Destination reached. Hovering...")
                self.target_pose_map = copy.deepcopy(self.current_map_pose)
                self.state = "HOVER"
        # <<< CHANGE END >>>

        elif self.state == "LAND":
            # For landing, we can either point the gimbal down or just leave it.
            # Pointing down: self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_MOUNT_CONTROL, param1=-90.0, param7=2.0) # MAV_MOUNT_MODE_MAVLINK_TARGETING
            self.get_logger().info("Mission: Landing at final destination.")
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.state = "MISSION_COMPLETE"
        
        elif self.state == "MISSION_COMPLETE":
            self.get_logger().info("Mission complete. Node is idle.", throttle_duration_sec=10)
            pass

def main(args=None):
    rclpy.init(args=args)
    mission_node = AbsoluteCoordMission()
    try:
        rclpy.spin(mission_node)
    except KeyboardInterrupt:
        mission_node.get_logger().info("User requested shutdown. Landing drone.")
        mission_node.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
    finally:
        mission_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
