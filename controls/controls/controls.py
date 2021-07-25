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
        self.velocity = Float32()
        self.max_angle1 = Float32()
        self.max_angle2 = Float32()
        self.max_angle3 = Float32()
        self.max_angle4 = Float32()
        self.angle1 = Float32()
        self.angle2 = Float32()
        self.angle3= Float32()
        self.angle4 = Float32()
    def angle_limiter(self):
        # self.max_angle1 = 0.0
        # max_angle2 = 0.0
        # max_angle3 = 0.0
        # max_angle4 = 0.0
        #turning towards left
        if(self.j.axes[2]>0):
            self.max_angle1.data = (70*math.pi/180)*math.exp(-0.28*abs(self.velocity.data))                             #front left-defined postive by convention
            self.max_angle2.data = (90*math.pi/180 - math.atan((math.cos(self.max_angle1.data)/(math.sin(self.max_angle1.data)+0.00001))+0.8374))
            self.max_angle3.data = -(self.max_angle1.data)
            self.max_angle4.data = -(self.max_angle2.data)

            """In case of a left turn, the left wheel(denoted by angle1 is constrained at 70 degrees
            and the right wheel takes an absolute value of less than 70(from relation)"""
        #turning towards right
        elif(self.j.axes[2]<=0): 
            self.max_angle2.data = ((70*math.pi/180)*math.exp(-0.28*abs(self.velocity.data)))
            self.max_angle1.data = (90*math.pi/180 - math.atan((math.cos(self.max_angle2.data)/(math.sin(self.max_angle2.data)+0.00001))+0.8374))
            self.max_angle3.data = -(self.max_angle1.data)
            self.max_angle4.data = -(self.max_angle2.data)
        
        
        self.angle1.data=self.angle_factor*self.max_angle1.data
        self.angle2.data=self.angle_factor*self.max_angle2.data
        self.angle3.data=self.angle_factor*self.max_angle3.data
        self.angle4.data=self.angle_factor*self.max_angle4.data

        self.angle1_pub.publish(self.angle1)
        self.angle2_pub.publish(self.angle2)
        self.angle3_pub.publish(self.angle3)
        self.angle4_pub.publish(self.angle4)


    def controller_callback(self,msg):
        self.j = msg
        self.velocity.data = self.j.axes[1]  
        self.velocity.data*=20
        #x = self.velocity
        self.angle_factor = self.j.axes[2]
        print (self.velocity)
        self.velocity_pub.publish(self.velocity)
        self.angle_limiter()
    

    

def main(args=None):
    rclpy.init(args=args)

    controller_subscriber = MinimalSubscriber()
    #controller_subscriber.velocity_pub.publish(controller_subscriber.velocity)
    #controller_subscriber.angle_limiter()
    rclpy.spin(controller_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
