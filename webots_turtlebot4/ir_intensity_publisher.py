import datetime

from irobot_create_msgs.msg import IrIntensityVector, IrIntensity

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Range


IR_SENSOR_SUBSCRIPTION_LIST = ["/Turtlebot4/ir_intensity_front_center_left", "/Turtlebot4/ir_intensity_front_center_right", "/Turtlebot4/ir_intensity_front_left", "/Turtlebot4/ir_intensity_front_right", "/Turtlebot4/ir_intensity_left", "/Turtlebot4/ir_intensity_right","/Turtlebot4/ir_intensity_side_left"]

class WebotsTurtleBot4IRPublisher(Node):

    def __init__(self):
        super().__init__('webots_turtlebot4_ir_intensity_publisher')

        self.publish_rate = 62
        
        self.msg = IrIntensityVector()
        self.msg.readings = []
        for _ in range(len(IR_SENSOR_SUBSCRIPTION_LIST)):
             self.msg.readings.append(IrIntensity())
        self.msg.header.frame_id = "base_link"

        # Create a publisher for the /cmd_lightring topic
        self.publisher = self.create_publisher(
            IrIntensityVector,
            '/ir_intensity',
            qos_profile_sensor_data)
        
        timer_period = 1 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.subscribers = []
        
        for ir_topic in IR_SENSOR_SUBSCRIPTION_LIST:
            self.subscribers.append(self.create_subscription(
            Range,
            ir_topic,
            self.listener_callback,
            10))
        

    def timer_callback(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.msg)

    # Interface buttons subscription callback
    def listener_callback(self, ir_msg):
        index = IR_SENSOR_SUBSCRIPTION_LIST.index(f"/Turtlebot4/{ir_msg.header.frame_id}")
        self.msg.readings[index].header.stamp = self.get_clock().now().to_msg()
        self.msg.readings[index].header.frame_id = ir_msg.header.frame_id
        self.msg.readings[index].value = int((ir_msg.range/0.10000000149011612)*32767)
        

def main(args=None):
    rclpy.init(args=args)
    node = WebotsTurtleBot4IRPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


