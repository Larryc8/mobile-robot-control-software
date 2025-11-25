#!/usr/bin/env python

import rospy
from numpy.linalg import norm
import math
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion

class PIDController:
    def __init__(self, kp, ki, kd, max_output, min_output, max_integral, tolerancia = 0.10):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.min_output = min_output
        self.max_integral = max_integral
        self.tolerancia = tolerancia
        
        self.previous_error = 0.0
        self.integral = 0.0
        self.last_time = None

    def set_setpoint(self, setpoint):
        pass
        
    def reset(self):
        self.previous_error = 0.0
        self.integral = 0.0
        self.last_time = None
        
    def compute(self, setpoint, current_value, current_time):
        error = setpoint - current_value
        if abs(error) < self.tolerancia:
            error = 0
            
        print('Error de la salida de control ', error, "tol:", self.tolerancia)
        
        # if self.last_time is None:
        #     self.last_time = current_time
        #     return -10
            
        # dt = current_time - self.last_time
        # if dt <= 0:
        #     return 0.0
            
        # Proportional term
        p_term = self.kp * error
        
        # Integral term
        self.integral += error 
        # Anti-windup
        if self.max_integral > 0:
            self.integral = max(min(self.integral, self.max_integral), -self.max_integral)
        i_term = self.ki * self.integral
        
        # Derivative term
        d_term = self.kd * (error - self.previous_error)
        
        # Compute output
        output = p_term  + i_term + d_term
        
        # Clamp output
        # output = max(min(output, self.max_output), self.min_output)
        
        # Update previous values
        self.previous_error = error
        self.last_time = current_time
        
        return output

class DifferentialDriveController:
    def __init__(self):
        
        # PID parameters for linear velocity
        linear_kp = 0.01#rospy.get_param('~linear_kp', 0.5)
        linear_ki = 0#rospy.get_param('~linear_ki', 0.01)
        linear_kd = 0#rospy.get_param('~linear_kd', 0.05)
        linear_max_output = 1 #rospy.get_param('~linear_max_output', 1.0)
        linear_min_output = 0.1#rospy.get_param('~linear_min_output', -1.0)
        linear_max_integral = 1#rospy.get_param('~linear_max_integral', 1.0)
        
        # PID parameters for yaw (angular) control
        yaw_kp = -0.4#rospy.get_param('~yaw_kp', 1.0)
        yaw_ki = 0#rospy.get_param('~yaw_ki', 0.01)
        yaw_kd = 0#rospy.get_param('~yaw_kd', 0.1)
        yaw_max_output = 1#rospy.get_param('~yaw_max_output', 2.0)
        yaw_min_output = 0.1#rospy.get_param('~yaw_min_output', -2.0)
        yaw_max_integral = 1#rospy.get_param('~yaw_max_integral', 1.0)

        self._markerIsVisible = False
        
        # Create PID controllers
        self.linear_pid = PIDController(linear_kp, linear_ki, linear_kd, 
                                      linear_max_output, linear_min_output, 
                                      linear_max_integral, 2)
        
        self.yaw_pid = PIDController(yaw_kp, yaw_ki, yaw_kd,
                                   yaw_max_output, yaw_min_output,
                                   yaw_max_integral, 0.02)
        
        # Current state
        self.current_pose = Pose2D()
        self.current_linear_velocity = 0.0
        self.current_angular_velocity = 0.0
        
        # Desired setpoints
        self.desired_pose = Pose2D()
        self.desired_yaw = 0.0
        
        # Publishers and Subscribers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=3)
        self.odom_sub = rospy.Subscriber('/aruco/odom', Odometry, self.odom_callback)
        self.marker_visible_sub = rospy.Subscriber('/aruco/visible', Bool, self.marker_callback)

        
        # You might want to subscribe to setpoint topics
        # For example:
        # self.velocity_setpoint_sub = rospy.Subscriber('/desired_velocity', Twist, self.velocity_setpoint_callback)
        # self.yaw_setpoint_sub = rospy.Subscriber('/desired_yaw', Float32, self.yaw_setpoint_callback)
        
        # Control rate
        self.control_rate = rospy.Rate(10)  # 50 Hz

    def marker_callback(self, msg):
        pass
        self._markerIsVisible = msg.data
        
    def odom_callback(self, odom_msg):
        # Extract position
        self.current_pose.x = odom_msg.pose.pose.position.x
        self.current_pose.y = odom_msg.pose.pose.position.y
        
        # Extract orientation (convert quaternion to Euler angles)
        orientation_q = odom_msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        roll, pitch, yaw = euler_from_quaternion(orientation_list)
        self.current_pose.theta = yaw
        
        # Extract velocities
        self.current_linear_velocity = 0 #odom_msg.twist.twist.linear.x
        self.current_angular_velocity = 0 #odom_msg.twist.twist.angular.z
        # rospy.loginfo("odom callback")
        # print(f"{__name__}: {yaw}")
        
    def velocity_setpoint_callback(self, twist_msg):
        self.desired_pose = twist_msg.linear.x
        self.desired_yaw = twist_msg.angular.z  # or use separate yaw setpoint
        
    def yaw_setpoint_callback(self, yaw_msg):
        self.desired_yaw = yaw_msg.data
        
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
        
    def control_loop(self, yaw = False):
        if not yaw:
            current_time = rospy.get_time()
            
            g = [self.desired_pose.x - self.current_pose.x, self.desired_pose.y - self.current_pose.y]
            WB = 1
            L = norm(g)
            gy = g[1]
            alpha = 2*WB*gy/(L**2)
            # Compute linear velocity control
            linear_control = self.linear_pid.compute(
                0, 
                L*math.copysign(1, g[0]), 
                current_time
            )
            print("y dellta: ", gy, alpha)

            
            # Compute yaw control (handle angle wrapping)
            # self.desired_yaw =  math.atan2(self.desired_pose.x - self.current_pose.x, self.desired_pose.y - self.current_pose.y)
            # current_yaw = self.normalize_angle(self.current_pose.theta)
            # desired_yaw = self.desired_yaw#self.normalize_angle(self.desired_yaw)
            #
            # # Calculate shortest path for yaw control
            # yaw_error =  desired_yaw - current_yaw#self.normalize_angle(desired_yaw - current_yaw)
            #
            # yaw_control = self.yaw_pid.compute(
            #     0,  # We want the error to be zero
            #     yaw_error,  # Use the normalized error as input
            #     current_time
            # )

            yaw_control = linear_control*alpha           
            cmd_vel = Twist()
            cmd_vel.linear.x = -linear_control
            cmd_vel.angular.z = yaw_control

            if not self._markerIsVisible:
                yaw_control = -0.1#*math.copysign(1, yaw_control)
                linear_control = 0#-linear_control

                cmd_vel = Twist()
                cmd_vel.linear.x = linear_control
                cmd_vel.angular.z = yaw_control

                self.cmd_vel_pub.publish(cmd_vel)
                return 10, 10
            
            # Create and publish control command
            
            self.cmd_vel_pub.publish(cmd_vel)

            return  yaw_control, linear_control
        else:

            current_time = rospy.get_time()
            current_yaw = self.current_pose.theta
            yaw_error =  self.desired_yaw - current_yaw#self.normalize_angle(desired_yaw - current_yaw)
            yaw_control = self.yaw_pid.compute(
                0,  # We want the error to be zero
                yaw_error,  # Use the normalized error as input
                current_time
            )
            
            print(f"curren yaw {current_yaw} target yaw {self.desired_yaw}")
            cmd_vel = Twist()
            cmd_vel.linear.x = 0
            cmd_vel.angular.z = yaw_control
            self.cmd_vel_pub.publish(cmd_vel)
            return yaw_control, 0
            # self.control_rate.sleep()
            
    def set_linear_distance_setpoint(self, target_pose):
        self.desired_pose.x = target_pose[0]
        self.desired_pose.y = target_pose[1]
        
    def set_yaw_setpoint(self, yaw):
        self.desired_yaw = yaw
        
    def reset_controllers(self):
        self.linear_pid.reset()
        self.yaw_pid.reset()

if __name__ == '__main__':
    try:
        rospy.init_node('differential_drive_controller')
        controller = DifferentialDriveController()
        
        # Example: Set some desired values (you would typically get these from other nodes)
        controller.set_linear_distance_setpoint((-2.2,0.1))  # 0.5 m/s
        controller.set_yaw_setpoint(-0.14)  # 90 degrees
        

        control_rate = rospy.Rate(10)  # 50 Hz

        while not rospy.is_shutdown():
            controller.control_loop()
            control_rate.sleep()
        
    except rospy.ROSInterruptException:
        pass
