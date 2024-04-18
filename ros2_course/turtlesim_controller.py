import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class TurtlesimController(Node):

    def __init__(self, speed, omega):
        super().__init__('turtlesim_controller')
        self.twist_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.pose = None
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.cb_pose,
            10
        )
        self.declare_parameter("iter", 2.5)
        self.timer = self.create_timer(5, self.param_callback)

        self.speed:float = speed
        self.omega:float = omega
        self.iter = None


    def param_callback(self):
        self.iter = self.get_parameter("iter").get_parameter_value().double_value

    def cb_pose(self, msg):
        self.pose = msg

    def go_straight(self, distance):
        """
        Move the robot straight at a specified speed for a given distance.

        Parameters:
            distance (float): The distance the robot should travel in meters. Positive value for forward motion, negative for backward.

        Notes:
            - This method blocks until the robot reaches the desired destination.
            - The robot's pose (position and orientation) must be available before calling this method.
            - The robot must have a functional ROS 2 environment with a Twist message publisher set up.
            - The method uses a proportional control (P-controller) to adjust the robot's speed based on the distance error.

        """
        # Wait for position to be received
        loop_rate = self.create_rate(100, self.get_clock()) # Hz
        while self.pose is None and rclpy.ok():
            self.get_logger().info('Waiting for pose...')
            rclpy.spin_once(self)
        
        # calculate the coordinate of the distance
        dx, dy = self._get_front_coordinates(self.pose.x, self.pose.y, self.pose.theta, distance)
        error = math.sqrt((dx - self.pose.x) ** 2 + (dy - self.pose.y) ** 2)
        is_front = self._is_destination_in_front(dx, dy)

        Kp = 1.5

        vel_msg = Twist()
        if distance > 0:
            vel_msg.linear.x = self.speed
        else:
            vel_msg.linear.x = -self.speed
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        vel_msg.angular.z = 0.0

        # Set loop rate
        loop_rate = self.create_rate(125, self.get_clock()) # Hz
        
        while abs(error) >= 0.01:
            error = math.sqrt((dx - self.pose.x) ** 2 + (dy - self.pose.y) ** 2)
            check_dest_front = self._is_destination_in_front(dx, dy)

            if is_front != check_dest_front:
                distance *= -1
                is_front = check_dest_front
                self.get_logger().info("Changed direction")

            # Calculate control signal
            control_signal = error * Kp
            control_signal = math.copysign(control_signal, distance)

            # Limit control signal to the maximum linear velocity
            control_signal = min(max(control_signal, -self.speed), self.speed)

            # Set linear velocity
            vel_msg.linear.x = control_signal

            # Publish Twist message
            self.twist_pub.publish(vel_msg)

            # Log information
            #self.get_logger().info("Current Distance: {:.2f}".format(error))
            #self.get_logger().info("Target Y: {:.2f}".format(self.pose.y))
            #self.get_logger().info("Control Signal: {:.2f}".format(control_signal))
            rclpy.spin_once(self)

        # turtle arrived, set velocity to 0
        vel_msg.linear.x = 0.0
        self.twist_pub.publish(vel_msg)
        #self.get_logger().info('Arrived to destination.')

    def turn(self, angle):
        """
        Turn the robot at a specified angular velocity to achieve a desired angle.

        Parameters:
            angle (float): The angle by which the robot should turn in degrees. Positive value for counterclockwise rotation, negative for clockwise.

        Notes:
            - This method blocks until the robot reaches the desired orientation.
            - The robot's pose (position and orientation) must be available before calling this method.
            - The robot must have a functional ROS 2 environment with a Twist message publisher set up.
            - The method uses a proportional control (P-controller) to adjust the robot's angular velocity based on the angle error.
            - The robot will stop when it reaches within 0.015 degrees of the target angle.

        """
        # Wait for position to be received
        loop_rate = self.create_rate(75, self.get_clock()) # Hz
        while self.pose is None and rclpy.ok():
            self.get_logger().info('Waiting for pose...')
            rclpy.spin_once(self)

        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        if angle > 0:
            vel_msg.angular.z = math.radians(self.omega)
        else:
            vel_msg.angular.z = -math.radians(self.omega)

        current_angle = math.degrees(self.pose.theta)
        current_angle = self._normalise_angle(current_angle)

        target_angle = current_angle + angle
        target_angle = self._normalise_angle(target_angle)

        Kp = 4

        # Set loop rate
        loop_rate = self.create_rate(100, self.get_clock()) # Hz

        error = target_angle - current_angle
        if abs(error) > 180:
            if error > 0:
                error -= 1 * 180
            else:
                error += 1 * 180
        
        while abs(error) > 0.015 and rclpy.ok():
            control_signal = error * Kp   

            control_signal = min(max(control_signal, -self.omega), self.omega)

            vel_msg.angular.z = math.radians(control_signal)

            self.twist_pub.publish(vel_msg)

            
            current_angle = math.degrees(self.pose.theta)
            current_angle = self._normalise_angle(current_angle)

            error = target_angle - current_angle
            if abs(error) > 180:
                if error > 0:
                    error -= 1 * 180
                else:
                    error += 1 * 180


            #self.get_logger().info("current angle: " + str(current_angle))
            #self.get_logger().info("target angle: " + str(target_angle))
            #self.get_logger().info("speed " + str(vel_msg.angular.z))
            rclpy.spin_once(self)
        

        # turtle arrived, set velocity to 0
        vel_msg.angular.z = 0.0
        self.twist_pub.publish(vel_msg)
        #self.get_logger().info('Arrived to destination.')
        
    def _get_front_coordinates(self, tx, ty, theta, distance):
        """
        Calculate the coordinates in front of the robot based on its orientation.

        Parameters:
            tx (float): The x-coordinate of the robot's current position.
            ty (float): The y-coordinate of the robot's current position.
            theta (float): The orientation angle of the robot in radians.
            distance (float): The distance from the current position to the front position.

        Returns:
            front_x (float): The x-coordinate of the position in front of the robot.
            front_y (float): The y-coordinate of the position in front of the robot.

        Notes:
            - This method calculates the coordinates in front of the robot based on its current position and orientation.
            - The distance parameter determines how far in front of the robot the coordinates will be calculated.
            - The orientation angle theta is assumed to be in radians.
        """
        # Calculate the x, y offset based on the turtle's orientation
        dx = distance * math.cos(theta)
        dy = distance * math.sin(theta)

        # Calculate the x, y coordinates in front of the turtle
        front_x = tx + dx
        front_y = ty + dy

        return front_x, front_y

    def _is_destination_in_front(self, dest_x, dest_y):
        """
        Determine if the destination point is in front of the robot.

        Parameters:
            dest_x (float): The x-coordinate of the destination point.
            dest_y (float): The y-coordinate of the destination point.

        Returns:
            in_front (bool): True if the destination point is in front of the robot, False otherwise.

        Notes:
            - This method calculates the angle between the robot's orientation and the vector pointing to the destination point.
            - If the absolute difference between this angle and the robot's orientation is less than 90 degrees, the destination is considered to be in front of the robot.
            - The angle is normalized to be between -180 and 180 degrees.
        """
        # Calculate the angle between turtle orientation and vector to destination
        angle_to_dest = math.atan2(dest_y- self.pose.y, dest_x - self.pose.x)
        angle_diff = math.degrees(abs(angle_to_dest - self.pose.theta))
        angle_diff = (angle_diff + 180) % 360 - 180  # Normalize angle between -180 and 180 degrees

        # Determine relative position
        if abs(angle_diff) < 90:
            self.get_logger().info("front")
            return True
        else:
            self.get_logger().info("Back")
            return False
    
    def _normalise_angle(self, angle):
        """
        Normalize the angle to be within the range [0, 360) degrees.

        Parameters:
            angle (float): The angle to be normalized.

        Returns:
            normalized_angle (float): The normalized angle within the range [0, 360) degrees.

        Notes:
            - This method ensures that the angle is within the range [0, 360) degrees by adjusting it accordingly.
            - Angles less than 0 are adjusted by adding 360 degrees.
            - Angles greater than or equal to 360 are adjusted by subtracting 360 degrees.
        """
        if angle < 0:
            angle += 360
        elif angle > 360:
            angle -= 360
        
        return angle

    def setup_turtle(self, offset):
        """
        Set up the turtle's initial position and speed.

        Parameters:
            offset (float): The offset distance from the starting position.

        Notes:
            - This method positions the turtle to the left and backward by 90 degrees and the specified offset distance.
            - If the turtle's speed is not set or set to a non-positive value, it is defaulted to 20 units.
        """
        self.turn(90)
        self.go_straight(-offset)

        if self.speed <= 0:
            self.speed = 20
    
    def draw_tree(self, i: int):
        """
        Recursively draw a tree-like structure using turtle graphics.

        Parameters:
            i (int): The current length of the branch.

        Notes:
            - This method recursively draws a tree-like structure using turtle graphics.
            - The parameter 'i' represents the current length of the branch.
            - The recursion stops when the length of the branch 'i' falls below 0.5 units.
            - At each step, the method draws a branch of length 'i' and then recursively calls itself to draw the sub-branches.
            - The angles for branching are set to 30 degrees to the left and 60 degrees to the right.
        """
        #print(i)
        if i < 0.5:
            return
        else:
            
            self.go_straight(i)
            next = 3*i/4
            if (next >= 0.5):
                self.turn(30.0)
                self.draw_tree(next)

                self.turn(-60.0)
                self.draw_tree(next)

                self.turn(30.0)

            self.go_straight(-i)

def main(args=None):
    rclpy.init(args=args)

    tc = TurtlesimController(6.0, 60.0)
    iterations = 2.5
    while tc.iter is None and rclpy.ok():
        rclpy.spin_once(tc)
        print(tc.iter)

    iterations = tc.iter

    tc.setup_turtle(iterations)
    tc.draw_tree(iterations)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    tc.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()