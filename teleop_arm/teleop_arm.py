#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import GripperCommand
from builtin_interfaces.msg import Duration
import math
import time

class SafeIntuitiveArmTeleop(Node):
    def __init__(self):
        super().__init__('safe_intuitive_arm_teleop')

        # QoS Profiles
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.base_pub = self.create_publisher(Twist, '/cmd_vel', qos_profile)
        self.joint_traj_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', qos_profile)
        
        # Gripper action client
        self.gripper_action_client = ActionClient(
            self, GripperCommand, '/gripper_controller/gripper_cmd')
        
        # Subscriptions
        self.create_subscription(Joy, '/joy', self.joy_callback, best_effort_qos)
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, qos_profile)

        # Robot state
        self.current_joint_positions = {}
        self.arm_joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        self.gripper_joint_names = ['gripper_left_joint', 'gripper_right_joint']
        
        # Control parameters
        self.base_linear_scale = 0.15   # Much slower base movement
        self.base_angular_scale = 0.3   # Much slower turning
        self.arm_increment = 0.1       # Smaller arm increments
        self.fine_increment = 0.005     # Very small fine increments
        self.gripper_open_pos = 0.019
        self.gripper_close_pos = -0.015
        self.deadzone = 0.35            # Even larger deadzone
        
        # Safety limits
        self.max_base_speed = 0.25      # Much slower max speed
        self.safety_timeout = 0.5
        
        # Joint limits
        self.joint_limits = {
            'joint1': (-3.14, 3.14),
            'joint2': (-1.57, 1.57),
            'joint3': (-1.57, 1.57),
            'joint4': (-3.14, 3.14)
        }
        
        # State tracking
        self.control_mode = 'BASE'
        self.gripper_open = True
        self.emergency_stop = False
        self.last_joy_time = time.time()
        self.last_buttons = []
        
        # Movement smoothing
        self.base_velocity_filter = VelocityFilter(alpha=0.8)
        
        # Arm control - COMPLETELY NEW APPROACH
        self.last_arm_command_time = 0
        self.arm_command_interval = 0.1  # Much slower rate
        self.movement_threshold = 0.5    # Much higher threshold
        self.last_log_time = 0
        self.log_interval = 3.0          # Log every 3 seconds max
        
        # DISCRETE movement approach - no continuous commands
        self.arm_moving = False
        self.last_joystick_state = {'joint1': 0, 'joint2': 0, 'joint3': 0, 'joint4': 0}
        
        # Joint holding positions - to prevent drift
        self.target_joint_positions = {}
        self.position_initialized = False
        
        # Create safety timer
        self.safety_timer = self.create_timer(0.1, self.safety_check)
        
        self.get_logger().info('🤖 TurtleBot3+Arm Teleop Ready!')
        self.get_logger().info('🎮 Controls:')
        self.get_logger().info('  Y Button: Switch BASE/ARM mode')
        self.get_logger().info('  BASE Mode: Left stick=move, Right stick=turn')
        self.get_logger().info('  ARM Mode:')
        self.get_logger().info('    Left stick X: joint1 (base rotation)')
        self.get_logger().info('    Left stick Y: joint2 (shoulder)')
        self.get_logger().info('    Right stick Y: joint3 (elbow)')
        self.get_logger().info('    Right stick X: joint4 (wrist)')
        self.get_logger().info('  L1/R1: Gripper open/close')
        self.get_logger().info('  Start: Home position')
        self.get_logger().info('  L2: Fine control mode')
        self.get_logger().info('  SELECT: Debug joint info')

        # Init the robot by bringing the ARM to Home and Gripper open
        self.send_gripper_command(self.gripper_open_pos, "OPEN")
        self.gripper_current_pos = self.gripper_open_pos

        self.move_to_home_pose()

    def joint_state_callback(self, msg: JointState):
        """Track current joint positions"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_positions[name] = msg.position[i]
        
        # Initialize target positions when first getting joint states
        if not self.position_initialized and all(joint in self.current_joint_positions for joint in self.arm_joint_names):
            for joint_name in self.arm_joint_names:
                self.target_joint_positions[joint_name] = self.current_joint_positions[joint_name]
            self.position_initialized = True
            self.get_logger().info('✅ Joint positions initialized')

    def joy_callback(self, msg: Joy):
        if len(msg.axes) < 4 or len(msg.buttons) < 8:
            return
            
        self.last_joy_time = time.time()
        
        # Debug joint info (SELECT button)
        if len(msg.buttons) > 8 and msg.buttons[8] and self.button_pressed(msg, 8):
            self.debug_joint_info()
        
        # Emergency stop check
        if (msg.buttons[4] and msg.buttons[5] and 
            len(msg.axes) > 5 and msg.axes[2] < -0.5 and msg.axes[5] < -0.5):
            if not self.emergency_stop:
                self.emergency_stop = True
                self.stop_all_movement()
                self.get_logger().warn('🚨 EMERGENCY STOP ACTIVATED!')
            return
        
        # Reset emergency stop
        if self.emergency_stop and not (msg.buttons[4] and msg.buttons[5]):
            self.emergency_stop = False
            self.get_logger().info('✅ Emergency stop cleared')
        
        # Mode switching (Y button)
        if self.button_pressed(msg, 3):
            old_mode = self.control_mode
            self.control_mode = 'ARM' if self.control_mode == 'BASE' else 'BASE'
            self.stop_all_movement()
            
            # When switching to ARM mode, initialize target positions to current positions
            if self.control_mode == 'ARM' and self.position_initialized:
                for joint_name in self.arm_joint_names:
                    self.target_joint_positions[joint_name] = self.current_joint_positions.get(joint_name, 0.0)
                self.get_logger().info(f'🔄 Switched to {self.control_mode} mode - positions locked')
            else:
                self.get_logger().info(f'🔄 Switched to {self.control_mode} mode')
        
        # Fine control mode (L2 trigger)
        fine_mode = len(msg.axes) > 2 and msg.axes[2] < -0.5
        
        # Process movement based on current mode
        if self.control_mode == 'BASE':
            self.handle_base_control(msg, fine_mode)
        elif self.control_mode == 'ARM':
            self.handle_arm_control_new(msg, fine_mode)
        
        # Gripper control
        self.handle_gripper_control(msg)
        
        # Home position (Start button)
        if self.button_pressed(msg, 8):
            self.move_to_home_pose()
        
        # Update button states
        self.last_buttons = list(msg.buttons)

    def debug_joint_info(self):
        """Debug current joint positions"""
        self.get_logger().info('🔍 CURRENT JOINT POSITIONS:')
        for name in self.arm_joint_names:
            current_pos = self.current_joint_positions.get(name, 0.0)
            target_pos = self.target_joint_positions.get(name, 0.0)
            self.get_logger().info(f'  {name}: current={current_pos:.3f} target={target_pos:.3f} ({math.degrees(current_pos):.1f}°)')

    def handle_base_control(self, msg: Joy, fine_mode: bool):
        """Handle TurtleBot3 base movement"""
        twist = Twist()
        
        # Speed scaling - much slower
        linear_scale = self.base_linear_scale * (0.2 if fine_mode else 1.0)  # Even slower in fine mode
        angular_scale = self.base_angular_scale * (0.2 if fine_mode else 1.0)
        
        # Left stick: forward/backward
        linear_x = self.apply_deadzone(msg.axes[1])
        
        # Right stick: turning
        angular_z = self.apply_deadzone(msg.axes[3])
        
        # Apply scaling and limits - more conservative
        twist.linear.x = self.clamp(linear_x * linear_scale, -self.max_base_speed, self.max_base_speed)
        twist.angular.z = self.clamp(angular_z * angular_scale, -1.0, 1.0)  # Slower max turn rate
        
        # Apply filtering
        filtered_twist = self.base_velocity_filter.filter(twist)
        
        # Publish if significant movement
        if self.has_significant_movement(filtered_twist):
            self.base_pub.publish(filtered_twist)

    def handle_arm_control_new(self, msg: Joy, fine_mode: bool):
        """DISCRETE ARM CONTROL: Only move when stick crosses threshold, then stop"""
        current_time = time.time()
        
        # Rate limit arm commands
        if current_time - self.last_arm_command_time < self.arm_command_interval:
            return
        
        if not self.position_initialized:
            return
        
        # Map joystick inputs to joints with MUCH higher threshold
        joint_inputs = {
            'joint1': msg.axes[0],  # Left stick X - no deadzone yet
            'joint2': msg.axes[1],  # Left stick Y
            'joint3': msg.axes[4],  # Right stick Y
            'joint4': msg.axes[3]   # Right stick X
        }
        
        # Detect NEW significant input (crossing threshold)
        movement_commands = {}
        for joint_name in self.arm_joint_names:
            current_input = joint_inputs[joint_name]
            last_input = self.last_joystick_state[joint_name]
            
            # Only trigger movement if crossing threshold from below
            if abs(current_input) > self.movement_threshold and abs(last_input) <= self.movement_threshold:
                # Determine direction
                direction = 1 if current_input > 0 else -1
                movement_commands[joint_name] = direction
                
                self.get_logger().info(f'🦾 {joint_name}: {"+" if direction > 0 else "-"} movement triggered')
        
        # Update joystick state tracking
        self.last_joystick_state = joint_inputs.copy()
        
        # Execute discrete movements
        if movement_commands:
            self.execute_discrete_arm_movement(movement_commands, fine_mode)
            self.last_arm_command_time = current_time
    
    def execute_discrete_arm_movement(self, movement_commands, fine_mode):
        """Execute discrete step movements for specified joints"""
        if not self.position_initialized:
            return
            
        # Movement increment
        increment = self.fine_increment if fine_mode else self.arm_increment
        
        # Calculate new target positions
        new_targets = {}
        for joint_name in self.arm_joint_names:
            current_target = self.target_joint_positions.get(joint_name, 0.0)
            
            if joint_name in movement_commands:
                delta = movement_commands[joint_name] * increment
                new_target = current_target + delta
                
                # Apply joint limits
                if joint_name in self.joint_limits:
                    min_limit, max_limit = self.joint_limits[joint_name]
                    new_target = self.clamp(new_target, min_limit, max_limit)
                
                new_targets[joint_name] = new_target
                self.target_joint_positions[joint_name] = new_target
                
                # Log the movement
                self.get_logger().info(f'🎯 {joint_name}: {current_target:.3f} → {new_target:.3f} rad')
        
        # Send single trajectory command
        if new_targets:
            self.send_discrete_joint_command()

    def send_discrete_joint_command(self):
        """Send ONE trajectory command for discrete movement"""
        traj_msg = JointTrajectory()
        traj_msg.header.stamp = self.get_clock().now().to_msg()
        traj_msg.joint_names = self.arm_joint_names
        
        target_positions = [self.target_joint_positions[joint] for joint in self.arm_joint_names]
        
        # Create trajectory point - single movement
        point = JointTrajectoryPoint()
        point.positions = target_positions
        point.velocities = [0.0] * len(target_positions)
        point.time_from_start = Duration(sec=1, nanosec=0)  # 1 second movement
        
        traj_msg.points = [point]
        self.joint_traj_pub.publish(traj_msg)
        
        self.get_logger().info('📤 Discrete arm movement command sent')

    def handle_gripper_control(self, msg: Joy):
        """Handle gripper control using action client in steps"""

        # Step size for each button press (adjust as needed)
        step = 0.01

        # Ensure current position is initialized
        if not hasattr(self, 'current_gripper_pos'):
            self.current_gripper_pos = self.gripper_open_pos  # Start at open position

        # L1 - Open in steps
        if self.button_pressed(msg, 4):
            self.current_gripper_pos += step
            if self.current_gripper_pos > self.gripper_open_pos:
                self.current_gripper_pos = self.gripper_open_pos
            self.send_gripper_command(self.current_gripper_pos, "OPEN")

        # R1 - Close in steps
        if self.button_pressed(msg, 5):
            self.current_gripper_pos -= step
            if self.current_gripper_pos < self.gripper_close_pos:
                self.current_gripper_pos = self.gripper_close_pos
            self.send_gripper_command(self.current_gripper_pos, "CLOSE")


    def send_gripper_command(self, position: float, action_name: str):
        """Send gripper command via action"""
        if not self.gripper_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('⚠️ Gripper action server not available')
            return
        
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = 10.0
        
        future = self.gripper_action_client.send_goal_async(goal_msg)
        self.get_logger().info(f'🫳 Gripper {action_name} command sent')

    def move_to_home_pose(self):
        """Move arm to home position"""
        self.stop_all_movement()
        
        # Home positions
        home_positions = [0.0, -1.0, 1.0, 0.0]
        
        # Update target positions
        for i, joint_name in enumerate(self.arm_joint_names):
            self.target_joint_positions[joint_name] = home_positions[i]
        
        # Send trajectory command
        traj_msg = JointTrajectory()
        traj_msg.header.stamp = self.get_clock().now().to_msg()
        traj_msg.joint_names = self.arm_joint_names
        
        point = JointTrajectoryPoint()
        point.positions = home_positions
        point.velocities = [0.0] * len(home_positions)
        point.time_from_start = Duration(sec=3, nanosec=0)
        
        traj_msg.points = [point]
        self.joint_traj_pub.publish(traj_msg)
        
        self.get_logger().info('🏠 Moving to home pose...')

    def safety_check(self):
        """Periodic safety checks"""
        current_time = time.time()
        
        # Stop if no joystick input
        if current_time - self.last_joy_time > self.safety_timeout:
            self.stop_all_movement()
        
        # Reduced status logging
        if int(current_time) % 60 == 0:  # Every 60 seconds
            mode_emoji = '🚗' if self.control_mode == 'BASE' else '🦾'
            self.get_logger().info(f'{mode_emoji} Mode: {self.control_mode}')

    def stop_all_movement(self):
        """Stop all movement"""
        self.base_pub.publish(Twist())

    def button_pressed(self, msg: Joy, button_index: int) -> bool:
        """Detect button press (edge detection)"""
        if len(msg.buttons) <= button_index or len(self.last_buttons) <= button_index:
            return False
        return msg.buttons[button_index] and not self.last_buttons[button_index]

    def apply_deadzone(self, value: float) -> float:
        """Apply deadzone to stick input"""
        return value if abs(value) > self.deadzone else 0.0

    def clamp(self, value: float, min_val: float, max_val: float) -> float:
        """Clamp value between min and max"""
        return max(min_val, min(max_val, value))

    def has_significant_movement(self, twist: Twist) -> bool:
        """Check if movement is significant"""
        return (abs(twist.linear.x) > 0.001 or 
                abs(twist.linear.y) > 0.001 or 
                abs(twist.angular.z) > 0.001)


class VelocityFilter:
    """Simple exponential moving average filter"""
    def __init__(self, alpha=0.7):
        self.alpha = alpha
        self.last_twist = None

    def filter(self, twist: Twist) -> Twist:
        if self.last_twist is None:
            self.last_twist = twist
            return twist
        
        filtered = Twist()
        filtered.linear.x = self.alpha * twist.linear.x + (1-self.alpha) * self.last_twist.linear.x
        filtered.linear.y = self.alpha * twist.linear.y + (1-self.alpha) * self.last_twist.linear.y
        filtered.angular.z = self.alpha * twist.angular.z + (1-self.alpha) * self.last_twist.angular.z
        
        self.last_twist = filtered
        return filtered


def main(args=None):
    rclpy.init(args=args)
    node = SafeIntuitiveArmTeleop()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Shutting down safely...')
        node.stop_all_movement()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()