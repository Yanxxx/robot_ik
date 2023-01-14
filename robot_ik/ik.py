import math

from geometry_msgs.msg import Twist
import geometry_msgs

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# from turtlesim.srv import Spawn

# import PyKDL
########### added 
class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(geometry_msgs.msg.Transform, 'topic',
                                    queue_size=5)
    def publish(self, msg):
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
########### added end
        
        
class FrameListener(Node):

    def __init__(self):
        super().__init__('vive_listener')

        # Declare and acquire `target_frame` parameter
        self.target_frame = self.declare_parameter(
          'target_frame', 'LeftHand').get_parameter_value().string_value
        print(self.target_frame)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


        self.timer = self.create_timer(0.03, self.on_timer)


    ##############added 
        self.msg = geometry_msgs.msg.Transform()  
        self.pub = MinimalPublisher()
    ############# added end

    def on_timer(self): 

        self.get_logger().info(
                f'timer called.')
        # while not rclpy._shutdown():            
        try:
            t = self.tf_buffer.lookup_transform(
                'world' ,
                'LeftHand',
                rclpy.time.Time())            
            # self.get_logger().info(
            #     f'Transformation Translation {t.transform.translation.x}, ' +
            #                 f'{t.transform.translation.y}, ' +
            #                 f'{t.transform.translation.z}, ' +
            #                 ' Rotation ' +
            #                 f'{t.transform.rotation.x}, ' +
            #                 f'{t.transform.rotation.y}, ' +
            #                 f'{t.transform.rotation.z}, ' +
            #                 f'{t.transform.rotation.w}, ' 
            #     )

            # print(
            #     f'Transformation Translation {t.transform.translation.x}, ' +
            #                 f'{t.transform.translation.y}, ' +
            #                 f'{t.transform.translation.z}, ' +
            #                 ' Rotation ' +
            #                 f'{t.transform.rotation.x}, ' +
            #                 f'{t.transform.rotation.y}, ' +
            #                 f'{t.transform.rotation.z}, ' +
            #                 f'{t.transform.rotation.w}, ' 
            #     )

            # ## added 
            self.msg.translation.x = t.transform.translation.x
            self.msg.translation.y = t.transform.translation.y
            self.msg.translation.z = t.transform.translation.z
            
            
            self.msg.rotation.x = t.transform.rotation.x
            self.msg.rotation.y = t.transform.rotation.y
            self.msg.rotation.z = t.transform.rotation.z
            self.msg.rotation.w = t.transform.rotation.w

            self.pub.publish(self.msg)
            ########### added end

        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform LeftHand to  world: {ex}')
            return



def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    
