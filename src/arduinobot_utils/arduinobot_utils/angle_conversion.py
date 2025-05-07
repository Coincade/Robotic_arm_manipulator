#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from arduinobot_msgs.srv import EulerToQuaternion, QuaternionToEuler
from tf_transformations import quaternion_from_euler, euler_from_quaternion

class AnglesConverter(Node):
    def __init__(self):
        super().__init__("angle_conversion_service_server")
        
        self.euler_to_quaternion_ = self.create_service(EulerToQuaternion, "euler_to_quaternion", self.eulerToQuaternionCallback)
        self.quaternion_to_euler_ = self.create_service(QuaternionToEuler, "quaternion_to_euler", self.quaternionToEulerCallback)
        self.get_logger().info("Angle Conversion Services are ready")
        
    def eulerToQuaternionCallback(self, req, res):
        self.get_logger().info(f"Requested to convert Euler's angle. roll: {req.roll}, pitch: {req.pitch}, yaw: {req.yaw} into a quaternion")
        (res.x, res.y, res.z, res.w) = quaternion_from_euler(req.roll, req.pitch, req.yaw)
        self.get_logger().info(f"Corresponding quaternion angles: x:{res.x}, y:{res.y}, z:{res.z}, w:{res.w}")
        return res
    
    def quaternionToEulerCallback(self, req, res):
        self.get_logger().info(f"Requested to convert Quaternion's angle. x: {req.x}, y: {req.y}, z: {req.z}, w:{req.w} into a Euler")
        (res.roll, res.pitch, res.yaw) = euler_from_quaternion([req.x, req.y, req.z, req.w])
        self.get_logger().info(f"Corresponding Euler angles: roll:{res.roll}, pitch:{res.pitch}, yaw:{res.yaw}")
        return res

def main():
    rclpy.init()
    angle_converter = AnglesConverter()
    rclpy.spin(angle_converter)
    angle_converter.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()    