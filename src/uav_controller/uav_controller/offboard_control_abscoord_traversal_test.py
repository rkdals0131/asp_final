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
    Node for offboard control using both absolute (map) and local coordinates.
    - Control loop is event-driven (triggered by local_position_callback) for data synchronization.
    - Takeoff uses local coordinates for stability, while the mission is performed using absolute map coordinates.
    - Mission: Fly a trajectory defined by waypoints, avoiding obstacles by flying at a safe cruise altitude.
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
        self.hover_duration = 50 # Corresponds to approx 5 seconds
        self.drone_frame_id = "x500_gimbal_0/base_link" # Make sure this matches your drone's frame
        self.next_state_after_hover = ""

        # --- Waypoint Mission --- in ENU system
        self.waypoints = [
            [-94.4088, 68.4708, 3.8531],     # marker 0
            [-75.4421, 74.9961, 23.2347],    # marker 1
            [-65.0308, 80.1275, 8.4990],     # marker 2
            [-82.7931, 113.4203, 3.8079],    # marker 3
            [-97.9238, 105.2799, 8.5504],    # marker 4
            [-109.1330, 100.3533, 23.1363]   # marker 5
        ]
        self.final_destination = [-62.9630, 99.0915, 0.1349]
        self.waypoint_index = 0
        # Calculate safe cruise altitude: max waypoint height + 10 meters for safety margin
        self.cruise_altitude = max(wp[2] for wp in self.waypoints) + 10.0
        self.waypoint_hover_altitude_offset = 5.0

        self.get_logger().info("Waypoint Mission Controller is running.")
        self.get_logger().info(f"Cruise altitude set to: {self.cruise_altitude:.2f}m")
        self.get_logger().info("Waiting for TF and Local Position data to start...")

    def local_position_callback(self, msg: VehicleLocalPosition):
        """Callback for VehicleLocalPosition, updates current local position and triggers the state machine."""
        self.current_local_pos = msg
        self.run_state_machine()

    def update_current_map_pose(self):
        """Update the drone's current absolute position in the 'map' frame using TF listener."""
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
        """Publishes the offboard control mode message."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_map_based_setpoint(self):
        """Calculates and publishes the trajectory setpoint based on the target map pose."""
        if self.current_map_pose is None or self.current_local_pos is None:
            return

        # Calculate the required change in the map frame
        delta_map_x = self.target_pose_map.pose.position.x - self.current_map_pose.pose.position.x
        delta_map_y = self.target_pose_map.pose.position.y - self.current_map_pose.pose.position.y
        delta_map_z = self.target_pose_map.pose.position.z - self.current_map_pose.pose.position.z
        
        # Convert map frame delta (FLU/ENU) to the drone's local NED frame delta
        # Gazebo's map is typically ENU (East, North, Up) or FLU (Forward, Left, Up) relative to world
        # PX4's local frame is NED (North, East, Down)
        # Transformation: map_x -> ned_y, map_y -> ned_x, map_z -> -ned_z
        delta_ned_x = delta_map_y
        delta_ned_y = delta_map_x
        delta_ned_z = -delta_map_z

        # Calculate the absolute target in the local NED frame
        target_ned_x = self.current_local_pos.x + delta_ned_x
        target_ned_y = self.current_local_pos.y + delta_ned_y
        target_ned_z = self.current_local_pos.z + delta_ned_z

        msg = TrajectorySetpoint()
        msg.position = [float(target_ned_x), float(target_ned_y), float(target_ned_z)]
        msg.yaw = 0.0  # Maintain North-facing yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)


    def publish_vehicle_command(self, command, **kwargs):
        """Publishes a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = kwargs.get("param1", 0.0)
        msg.param2 = kwargs.get("param2", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def check_arrival(self, tolerance=0.8):
        """Check if the drone has arrived at the target map position."""
        if self.current_map_pose is None: return False
        
        pos = self.current_map_pose.pose.position
        target_pos = self.target_pose_map.pose.position
        
        # For altitude-only movements, check only Z. For others, check X, Y, Z.
        if self.state in ["CRUISE_ALTITUDE"]:
            return abs(pos.z - target_pos.z) < tolerance
        
        distance = math.sqrt((pos.x - target_pos.x)**2 + (pos.y - target_pos.y)**2 + (pos.z - target_pos.z)**2)
        return distance < tolerance

    def run_state_machine(self):
        """Main state machine, driven by the local_position_callback."""
        if not self.update_current_map_pose() or self.current_local_pos is None:
            # Wait until both TF and local position are available
            return

        # Publish current state
        state_msg = String()
        state_msg.data = self.state
        self.state_publisher.publish(state_msg)

        # Continuously publish offboard control mode except when landing/landed
        if self.state not in ["LAND", "MISSION_COMPLETE"]:
             self.publish_offboard_control_mode()

        # --- State Logic ---
        if self.state == "INIT":
            self.get_logger().info("TF and Local Position received. Starting handshake.")
            self.state = "HANDSHAKE"

        elif self.state == "HANDSHAKE":
            # Continuously send current position as setpoint to establish offboard link
            sp_msg = TrajectorySetpoint()
            sp_msg.position = [self.current_local_pos.x, self.current_local_pos.y, self.current_local_pos.z]
            sp_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.trajectory_setpoint_publisher.publish(sp_msg)
            
            self.handshake_counter += 1
            if self.handshake_counter > self.handshake_duration:
                self.get_logger().info("Handshake complete. Switching to Offboard mode and Arming.")
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0) # 6 for Offboard
                self.state = "ARMING"

        elif self.state == "ARMING":
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0) # Arm
            self.get_logger().info("Arm command sent.")
            self.get_logger().info(f"Takeoff initiated from MAP [x: {self.current_map_pose.pose.position.x:.2f}, y: {self.current_map_pose.pose.position.y:.2f}, z: {self.current_map_pose.pose.position.z:.2f}]")
            # Takeoff to 10m above current ground position using local coordinates for safety
            takeoff_altitude = 10.0
            self.takeoff_target_local = [self.current_local_pos.x, self.current_local_pos.y, self.current_local_pos.z - takeoff_altitude]
            self.state = "TAKEOFF"
        
        elif self.state == "TAKEOFF":
            sp_msg = TrajectorySetpoint()
            sp_msg.position = [float(p) for p in self.takeoff_target_local]
            sp_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.trajectory_setpoint_publisher.publish(sp_msg)
            
            # Check for takeoff completion (95% of target altitude)
            if abs(self.current_local_pos.z - self.takeoff_target_local[2]) < 1.0:
                self.get_logger().info("Takeoff complete. Preparing for mission.")
                self.target_pose_map = copy.deepcopy(self.current_map_pose) # Set hover target
                self.state = "HOVER"
                self.next_state_after_hover = "CRUISE_ALTITUDE"
                self.hover_counter = 0

        elif self.state == "HOVER":
            self.publish_map_based_setpoint()
            self.hover_counter += 1
            if self.hover_counter > self.hover_duration:
                self.get_logger().info(f"Hover complete. Proceeding to: {self.next_state_after_hover}")
                self.state = self.next_state_after_hover
                self.hover_counter = 0
        
        elif self.state == "CRUISE_ALTITUDE":
            self.get_logger().info(f"Ascending to safe cruise altitude: {self.cruise_altitude:.2f}m")
            self.target_pose_map.pose.position.z = self.cruise_altitude
            self.state = "EXECUTING_MOVE"
            # After reaching cruise altitude, move above the first waypoint
            self.next_state_after_hover = "MOVE_TO_WAYPOINT_ABOVE" 

        elif self.state == "MOVE_TO_WAYPOINT_ABOVE":
            wp = self.waypoints[self.waypoint_index]
            self.get_logger().info(f"Mission: Moving to above Waypoint #{self.waypoint_index + 1} at ({wp[0]:.2f}, {wp[1]:.2f}) at cruise altitude.")
            self.target_pose_map.pose.position.x = wp[0]
            self.target_pose_map.pose.position.y = wp[1]
            self.target_pose_map.pose.position.z = self.cruise_altitude
            self.state = "EXECUTING_MOVE"
            self.next_state_after_hover = "DESCEND_TO_WAYPOINT"

        elif self.state == "DESCEND_TO_WAYPOINT":
            wp = self.waypoints[self.waypoint_index]
            target_z = wp[2] + self.waypoint_hover_altitude_offset
            self.get_logger().info(f"Mission: Descending over Waypoint #{self.waypoint_index + 1} to altitude {target_z:.2f}m.")
            self.target_pose_map.pose.position.z = target_z
            self.state = "EXECUTING_MOVE"
            # After descending, increment waypoint and decide next step
            self.waypoint_index += 1
            if self.waypoint_index < len(self.waypoints):
                self.next_state_after_hover = "CRUISE_ALTITUDE" # Go back up for next waypoint
            else:
                self.next_state_after_hover = "MOVE_TO_FINAL_DESTINATION" # All waypoints visited

        elif self.state == "MOVE_TO_FINAL_DESTINATION":
            dest = self.final_destination
            self.get_logger().info(f"Mission: All waypoints visited. Moving to final destination at ({dest[0]:.2f}, {dest[1]:.2f}, {dest[2]:.2f}).")
            self.target_pose_map.pose.position.x = dest[0]
            self.target_pose_map.pose.position.y = dest[1]
            self.target_pose_map.pose.position.z = self.cruise_altitude # Approach at cruise alt first
            self.state = "EXECUTING_MOVE"
            self.next_state_after_hover = "LAND"

        elif self.state == "EXECUTING_MOVE":
            self.publish_map_based_setpoint()
            if self.check_arrival():
                self.get_logger().info(f"Destination reached. Hovering...")
                # Update target to current pose to ensure stable hover
                self.target_pose_map = copy.deepcopy(self.current_map_pose)
                self.state = "HOVER"

        elif self.state == "LAND":
            self.get_logger().info("Mission: Landing at final destination.")
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.state = "MISSION_COMPLETE"
        
        elif self.state == "MISSION_COMPLETE":
            self.get_logger().info("Mission complete. Node is idle.", throttle_duration_sec=10)
            pass # Do nothing, let the drone land and disarm automatically

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
