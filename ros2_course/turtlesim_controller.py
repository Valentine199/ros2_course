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

    def go_to(self, speed, omega, x, y):
        # Wait for position to be received
        loop_rate = self.create_rate(75, self.get_clock()) # Hz
        while self.pose is None and rclpy.ok():
            self.get_logger().info('Waiting for pose...')
            rclpy.spin_once(self)

        # Stuff with atan2
        x0 = self.pose.x
        y0 = self.pose.y
        theta_0 = math.degrees(self.pose.theta)

        theta_1 = math.degrees(math.atan2(y-y0, x-x0))
        angle = theta_1 - theta_0
        distance = math.sqrt(((x - x0) * (x - x0)) + (y - y0) * (y - y0))

        # Execute movement
        self.turn(omega, angle)
        self.go_straight(speed, distance)

    def go_straight(self, speed, distance):
        # Implement straght motion here
        # Create and publish msg

        # Wait for position to be received
        loop_rate = self.create_rate(100, self.get_clock()) # Hz
        while self.pose is None and rclpy.ok():
            self.get_logger().info('Waiting for pose...')
            rclpy.spin_once(self)
        
        dx, dy = self.get_front_coordinates(self.pose.x, self.pose.y, self.pose.theta, distance)
        current_distance = math.sqrt((dx - self.pose.x) ** 2 + (dy - self.pose.y) ** 2)
        target_distance = current_distance - abs(distance)  # Subtract distance instead of adding


        Kp = 1.15

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

        # Calculate time
        #T = abs(distance / speed)

        # Publish first msg and note time when to stop
        #self.twist_pub.publish(vel_msg)
        #self.get_logger().info('Turtle started.')
        #when = self.get_clock().now() + rclpy.time.Duration(seconds=T)

        # Publish msg while the calculated time is up
        #while (self.get_clock().now() <= when) and rclpy.ok():
            #self.twist_pub.publish(vel_msg)
            #self.get_logger().info('On its way...')
            #rclpy.spin_once(self)   # loop rate
        
        while True:
            rclpy.spin_once(self)
            #dx, dy = self.get_front_coordinates(self.pose.x, self.pose.y, self.pose.theta, distance)
            error = math.sqrt((dx - self.pose.x) ** 2 + (dy - self.pose.y) ** 2)
            # = distance - current_distance
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
            self.get_logger().info("Current Distance: {:.2f}".format(error))
            self.get_logger().info("Target Y: {:.2f}".format(self.pose.y))
            self.get_logger().info("Control Signal: {:.2f}".format(control_signal))

            # Check if within tolerance
            if abs(error) < 0.075:
                break

            
            # Sleep to maintain loop rate
            

        # turtle arrived, set velocity to 0
        vel_msg.linear.x = 0.0
        self.twist_pub.publish(vel_msg)
        #self.get_logger().info('Arrived to destination.')

    def turn(self, omega, angle):
        # Implement straght motion here
        # Create and publish msg

        last_angle = 0.0

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
        if current_angle < 0:
            current_angle += 2 * 180
        elif current_angle > 360:
            current_angle -= 2 * 180

        target_angle = current_angle + angle

        if target_angle < 0:
            target_angle += 2 * 180
        elif target_angle > 360:
            target_angle -= 2 * 180

        Kp = 0.05

        # Set loop rate
        loop_rate = self.create_rate(100, self.get_clock()) # Hz

        # Calculate time
        #T = abs(angle / omega)

        # Publish first msg and note time when to stop
        
        #self.get_logger().info('Turtle started.')
        #when = self.get_clock().now() + rclpy.time.Duration(seconds=T)
        

        # Publish msg while the calculated time is up
        #while (self.get_clock().now() <= when) and rclpy.ok():
            #Đself.twist_pub.publish(vel_msg)
            #self.get_logger().info('On its way...')
            #rclpy.spin_once(self)   # loop rate
        error = target_angle - current_angle
        if abs(error) > 180:
            if error > 0:
                error -= 2 * 180
            else:
                error += 2 * 180
        
        while abs(error) > 0.015 and rclpy.ok():
                   

            control_signal = error * Kp   

            control_signal = min(max(control_signal, -omega), omega)

            vel_msg.angular.z = control_signal

            self.twist_pub.publish(vel_msg)

            
            current_angle = math.degrees(self.pose.theta)
            if current_angle < 0:
                current_angle += 2 * 180
            elif current_angle > 360:
                current_angle -= 2 * 180

            error = target_angle - current_angle
            if abs(error) > 180:
                if error > 0:
                    error -= 2 * 180
                else:
                    error += 2 * 180

            # Sleep to maintain loop rate
            self.get_logger().info("current angle: " + str(current_angle))
            self.get_logger().info("target angle: " + str(target_angle))
            self.get_logger().info("speed " + str(vel_msg.angular.z))
            rclpy.spin_once(self)   # loop rate
        

        # turtle arrived, set velocity to 0
        vel_msg.angular.z = 0.0
        self.twist_pub.publish(vel_msg)
        #self.get_logger().info('Arrived to destination.')
        
    def get_front_coordinates(self, tx, ty, theta, distance):
        # Calculate the x, y offset based on the turtle's orientation
        dx = distance * math.cos(theta)
        dy = distance * math.sin(theta)

        # Calculate the x, y coordinates in front of the turtle
        front_x = tx + dx
        front_y = ty + dy

        return front_x, front_y

    def draw_poly(self, speed, omega, N, a):

        angle = 360.0 / N
        for i in range(N):
            self.go_straight(speed, a)
            self.turn(omega, angle)

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
    #tc = TurtlesimController()

    #tc.go_to(4.0, 40.0, 2, 8)
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