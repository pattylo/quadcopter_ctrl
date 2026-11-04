#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

def yaw_from_quat(x, y, z, w):
    """Extract yaw angle from quaternion"""
    s1 = 2.0 * (w * z + x * y)
    s2 = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(s1, s2)

def quat_from_yaw(yaw):
    """Generate quaternion from yaw angle"""
    h = 0.5 * yaw
    return (0.0, 0.0, math.sin(h), math.cos(h))

class offboardHover(Node):
    def __init__(self):
        super().__init__('offboard_hover')
        
        qos = QoSProfile(depth=10); 
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        
        self.get_logger().info("Circle flight example")

        self.declare_parameter('ns', 'mavros')
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('hover_z', 3.0)  # Takeoff altitude
        self.declare_parameter('circle_radius', 1.0)  # Circle radius
        self.declare_parameter('circle_speed', 0.5)  # Circular flight speed
        self.declare_parameter('descent_rate', 0.5)  # Descent speed (m/s)

        ns = self.get_parameter('ns').get_parameter_value().string_value
        self.ns = ns if ns.startswith('/') else '/' + ns
        self.rate = float(self.get_parameter('rate_hz').value)
        self.dt = 1.0 / self.rate
        self.hover_z = float(self.get_parameter('hover_z').value)
        self.circle_radius = float(self.get_parameter('circle_radius').value)
        self.circle_speed = float(self.get_parameter('circle_speed').value)
        self.descent_rate = float(self.get_parameter('descent_rate').value)

        # subscribers and publishers
        self.state = State()
        self.current_pose = PoseStamped()
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, 10)
        self.sp_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_callback, qos)

        # services
        self.srv_mode = self.create_client(SetMode, '/mavros/set_mode')
        self.srv_arm = self.create_client(CommandBool, '/mavros/cmd/arming')

        self.fsm = 'WARMUP'
        self.warmup_ticks = int(2.0 * self.rate)
        self.ticks = 0

        # Takeoff point position
        self.home_x = 0.0
        self.home_y = 0.0
        self.home_z = 0.0
        self.has_home = False
        self.init_yaw = 0.0

        # Circular flight state
        self.theta = 0.0  # Current angle
        self.angular_velocity = self.circle_speed / self.circle_radius  # rad/s
        
        # Landing position (where circle completes)
        self.land_x = 0.0
        self.land_y = 0.0
        self.target_altitude = self.hover_z  # Current target altitude

        # Timer
        self.timer = self.create_timer(self.dt, self.main_timer)
        self.get_logger().info(f"Circle flight: radius={self.circle_radius}m, speed={self.circle_speed}m/s")
        self.get_logger().info("Waiting for FCU connection...")

    def state_callback(self, msg: State):
        self.state = msg
        
    def pose_callback(self, msg: PoseStamped):
        self.current_pose = msg
        
        # Record takeoff point position (first time)
        if not self.has_home:
            self.home_x = msg.pose.position.x
            self.home_y = msg.pose.position.y
            self.home_z = msg.pose.position.z
            
            # Record initial yaw
            o = msg.pose.orientation
            self.init_yaw = yaw_from_quat(o.x, o.y, o.z, o.w)
            
            self.has_home = True
            self.get_logger().info(f"Takeoff point: ({self.home_x:.2f}, {self.home_y:.2f}, {self.home_z:.2f})")

    def set_mode(self, mode: str):
        if self.srv_mode.service_is_ready():
            req = SetMode.Request()
            req.custom_mode = mode
            self.srv_mode.call_async(req)

    def set_arm(self):
        if self.srv_arm.service_is_ready():
            req = CommandBool.Request()
            req.value = True
            self.srv_arm.call_async(req)
    
    def set_disarm(self):
        if self.srv_arm.service_is_ready():
            req = CommandBool.Request()
            req.value = False
            self.srv_arm.call_async(req)
    
    def get_circle_position(self):
        """Calculate position on the circle"""
        x = self.home_x + self.circle_radius * math.cos(self.theta)
        y = self.home_y + self.circle_radius * math.sin(self.theta)
        return x, y

    def main_timer(self):
        # Publish a constant position setpoint every tick
        
        sp = PoseStamped()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.header.frame_id = 'map'
        
        # Default hover position (above takeoff point)
        target_x = self.home_x
        target_y = self.home_y
        target_z = self.hover_z

        if not self.state.connected or not self.has_home:
            sp.pose.position.x = target_x
            sp.pose.position.y = target_y
            sp.pose.position.z = target_z
            sp.pose.orientation.w = 1.0
            self.sp_pub.publish(sp)
            return

        if self.fsm == 'WARMUP':
            self.ticks += 1

            if self.ticks >= self.warmup_ticks:
                self.set_mode("OFFBOARD")
                self.fsm = 'OFFBOARD'
                self.get_logger().info("→ OFFBOARD mode requested")

        elif self.fsm == 'OFFBOARD':
            if self.state.mode == "OFFBOARD":
                self.set_arm()
                self.fsm = 'ARM'
                self.get_logger().info("OFFBOARD confirmed → ARM requested")

        elif self.fsm == 'ARM':
            if self.state.armed:
                self.fsm = 'HOVER'
                self.get_logger().info("ARMED → HOVERING at (0,0,{:.2f})".format(self.hover_z))
        
        elif self.fsm == 'HOVER':
            # Wait for reaching hover altitude
            if abs(self.current_pose.pose.position.z - self.hover_z) < 0.3:
                self.fsm = 'CIRCLE'
                self.theta = 0.0
                self.get_logger().info("Reached hover altitude → Starting circle")
            
        elif self.fsm == 'CIRCLE':
            # Circular flight
            target_x, target_y = self.get_circle_position()
            
            # Update angle
            self.theta += self.angular_velocity * self.dt
            
            # Check if one circle is completed
            if self.theta >= 2 * math.pi:
                # Record current position for landing
                self.land_x = target_x
                self.land_y = target_y
                self.target_altitude = self.hover_z  # Start descending from hover altitude
                self.fsm = 'DESCEND'
                self.get_logger().info(f"Circle completed → Descending at ({self.land_x:.2f}, {self.land_y:.2f})")

        elif self.fsm == 'DESCEND':
            # Descend at current position with controlled rate
            target_x = self.land_x
            target_y = self.land_y
            
            # Gradually reduce target altitude
            descent_step = self.descent_rate * self.dt
            if self.target_altitude > self.home_z:
                self.target_altitude = max(self.target_altitude - descent_step, self.home_z)
            
            target_z = self.target_altitude
            
            # Check if reached ground
            if abs(self.current_pose.pose.position.z - self.home_z) < 0.3:
                self.set_disarm()
                self.fsm = 'DISARM'
                self.get_logger().info("Reached ground → Disarming")

        elif self.fsm == 'DISARM':
            # Wait for disarm
            target_x = self.land_x
            target_y = self.land_y
            target_z = self.home_z
            if not self.state.armed:
                self.fsm = 'DONE'
                self.get_logger().info("Disarmed → Task finished")

        elif self.fsm == 'DONE':
            # Hold position
            target_x = self.land_x
            target_y = self.land_y
            target_z = self.home_z
            pass

        # Set target position
        sp.pose.position.x = target_x
        sp.pose.position.y = target_y
        sp.pose.position.z = target_z
        
        # Maintain initial heading
        qx, qy, qz, qw = quat_from_yaw(self.init_yaw)
        sp.pose.orientation.x = qx
        sp.pose.orientation.y = qy
        sp.pose.orientation.z = qz
        sp.pose.orientation.w = qw
        
        self.sp_pub.publish(sp)

def main():
    rclpy.init()
    node = offboardHover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
