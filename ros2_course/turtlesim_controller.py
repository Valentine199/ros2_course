import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class TurtlesimController(Node):

    def __init__(self):
        super().__init__('turtlesim_controller')
        self.twist_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.pose = None
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.cb_pose,
            10
        )

    def cb_pose(self, msg):
        self.pose = msg

    def go_straight(self, speed, distance):
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
            vel_msg.linear.x = speed
        else:
            vel_msg.linear.x = -speed
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
            control_signal = min(max(control_signal, -speed), speed)

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

    def turn(self, omega, angle):
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
            vel_msg.angular.z = math.radians(omega)
        else:
            vel_msg.angular.z = -math.radians(omega)

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

            control_signal = min(max(control_signal, -omega), omega)

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
        # Calculate the x, y offset based on the turtle's orientation
        dx = distance * math.cos(theta)
        dy = distance * math.sin(theta)

        # Calculate the x, y coordinates in front of the turtle
        front_x = tx + dx
        front_y = ty + dy

        return front_x, front_y

    def _is_destination_in_front(self, dest_x, dest_y):
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
        if angle < 0:
            angle += 360
        elif angle > 360:
            angle -= 360
        
        return angle


class FractalTree():
    def __init__(self, speed, omega):
        self.turtle = TurtlesimController()
        self.speed:float = speed
        self.omega:float = omega
    
    def set_speed(self, speed: float):
        self.speed = speed

    def set_omega(self, omega: float):
        self.omega = omega
    
    def delete_turtle(self):
        self.turtle.destroy_node()
    
    def _forward(self, distance: int):
        distance = abs(distance)
        self.turtle.go_straight(self.speed, distance)
    
    def _backward(self, distance: int):
        if distance > 0:
            distance *= -1
        
        self.turtle.go_straight(self.speed, distance)
    
    def _left(self, angle: float):
        self.turtle.turn(self.omega, angle)
    
    def _right(self, angle: float):
        if angle > 0:
            angle *= -1
        self.turtle.turn(self.omega, angle)
    
    def setup_turtle(self, offset):
        self._left(90)
        self._backward(offset)

        if self.speed <= 0:
            self.speed = 20
    
    def draw_tree(self, i: int):
        #print(i)
        if i < 0.5:
            return
        else:
            
            self._forward(i)
            next = 3*i/4
            if (next >= 0.5):
                self._left(30.0)
                self.draw_tree(next)

                self._right(60.0)
                self.draw_tree(next)

                self._left(30.0)

            self._backward(i)




def main(args=None):
    rclpy.init(args=args)
    tc = FractalTree(6.0, 60.0)

    v = 2.5
    tc.setup_turtle(v)
    tc.draw_tree(v)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    #tc.destroy_node()
    tc.delete_turtle()
    rclpy.shutdown()

if __name__ == '__main__':
    main()