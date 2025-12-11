import rclpy
import rclpy.qos
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.node import Node

from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import String
from sensor_msgs.msg import Image
from vla_msgs.msg import CartesianDelta

from cv_bridge import CvBridge

# API interface stuff
import requests
import json_numpy
json_numpy.patch()
import numpy as np

import time
import math

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

class VLANode(Node):
    def __init__(self):
        super().__init__('vla_node')

        self.url = (
            self.declare_parameter('url', 'http://127.0.0.1:8000/act', ParameterDescriptor(description='OpenVLA API endpoint URL'))
                .get_parameter_value().string_value
        )
        self.unnorm_key = (
            self.declare_parameter('unnorm_key', 'roboturk', ParameterDescriptor(description='Unnormalisation key to be used for OpenVLA'))
                .get_parameter_value().string_value
        )
        
        self.image_sub = self.create_subscription(
            Image, 'image', # camera image topic - to be remapped
            self.image_cb,
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, depth=1) # discard missed messages
        )
        self.bridge = CvBridge()
        
        self.instr_sub = self.create_subscription(
            String, 'instruction', # instruction prompt
            self.instr_cb, rclpy.qos.qos_profile_system_default
        )
        self.instruction = ''

        self.output_pub = self.create_publisher(
            CartesianDelta, 'output', 1
        )
    
    def instr_cb(self, msg: String):
        self.instruction = msg.data
        if len(self.instruction) > 0:
            self.get_logger().info(f'received new instruction: {self.instruction}')
        else:
            self.get_logger().info(f'cleared instruction')
    
    def image_cb(self, msg: Image):
        if len(self.instruction) == 0: 
            self.get_logger().warn('no instructions yet - not running inference', throttle_duration_sec=1.0)
            return # no instruction

        img = self.bridge.imgmsg_to_cv2(msg) # image to numpy array - TODO: check dimensions

        # self.get_logger().info(f'Data types: image: {type(img)}, instruction: {type(self.instruction)}')

        t_start = time.time()
        action: np.ndarray = requests.post(
            self.url,
            json={
                'image': img, 'instruction': self.instruction,
                'unnorm_key': self.unnorm_key
            }
        ).json()
        action = action.tolist() # convert to Python floats
        t_end = time.time()
        self.get_logger().info(f'inference completed in {(t_end - t_start):.4f} sec')

        # self.get_logger().info(f'OpenVLA API response ({(t_end - t_start):.4f} sec): {action}')

        # publish output
        delta = CartesianDelta()
        delta.header = msg.header

        delta.transform.translation.x, delta.transform.translation.y, delta.transform.translation.z = action[:3]

        q = quaternion_from_euler(action[3], action[4], action[5])
        delta.transform.rotation.x, delta.transform.rotation.y, delta.transform.rotation.z, delta.transform.rotation.w = q

        delta.gripper = action[6]

        self.output_pub.publish(delta)


def main(args=None):
    rclpy.init(args=args)

    node = VLANode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
