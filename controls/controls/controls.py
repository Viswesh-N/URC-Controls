import rclpy
from rclpy.node import Node
import math

from std_msgs.msg import String
from std_msgs.msg import Float32
from sensor_msgs.msg import Joy

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('controller_subscriber')                                     #the __init__ is a constructor which initializes the name of the subscriber
        self.subscription = self.create_subscription(Joy,'/joy', self.controller_callback,10)
        self.velocity_pub = self.create_publisher(Float32, '/velocity', 10)
        self.angle1_pub = self.create_publisher(Float32, '/Front_left',10)
        self.angle2_pub = self.create_publisher(Float32, '/Front_right',10)
        self.angle3_pub = self.create_publisher(Float32, '/Back_left',10)
        self.angle4_pub = self.create_publisher(Float32, '/Back_right',10)
        self.subscription  # prevent unused variable warning
        self.j = Joy()
    def update(self, msg):
        self.j = msg

    def controller_callback(self):
        self.velocity = self.j.axes[1]  
        self.velocity*=float(math.pow(2,63))
        x = self.velocity
        angle_factor = self.j.axes[2]
        self.velocity_pub.publish(self.velocity)
    def angle_limiter(self):
        #turning towards left
        if(self.j.axes[2]<0):
            max_angle1 = (70*math.pi/180)*math.exp(-0.28*abs(self.velocity))                             #front left-defined postive by convention
            max_angle2 = (90*math.pi/180 - math.atan((math.cos(max_angle1)/math.sin(max_angle1))+0.8374))
            max_angle3 = -max_angle1
            max_angle4 = -max_angle2

            """In case of a left turn, the left wheel(denoted by angle1 is constrained at 70 degrees
            and the right wheel takes an absolute value of less than 70(from relation)"""
        #turning towards right
        elif(self.j.axes[2]>0): 
            max_angle2 = -((70*math.pi/180)*math.exp(-0.28*abs(self.velocity)))
            max_angle1 = -(90*math.pi/180 - math.atan((math.cos(max_angle2)/math.sin(max_angle2))+0.8374))
            max_angle3 = -max_angle1
            max_angle4 = -max_angle2
        

        self.angle1_pub.publish(self.j.axes[2]*max_angle1)
        self.angle2_pub.publish(self.j.axes[2]*max_angle2)
        self.angle3_pub.publish(self.j.axes[2]*max_angle3)
        self.angle4_pub.publish(self.j.axes[2]*max_angle4)


    

def main(args=None):
    rclpy.init(args=args)

    controller_subscriber = MinimalSubscriber()

    rclpy.spin_once(controller_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()