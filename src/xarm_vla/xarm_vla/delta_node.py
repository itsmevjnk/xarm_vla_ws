import rclpy
from rclpy.node import Node

# hack to call service from callback: https://gist.github.com/driftregion/14f6da05a71a57ef0804b68e17b06de5
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from threading import Event

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from rcl_interfaces.msg import ParameterDescriptor
from vla_msgs.msg import CartesianDelta
from xarm_msgs.srv import MoveCartesian, SetInt16, SetInt16ById, GripperMove # using SDK move function

import time

from scipy.spatial.transform import Rotation

def clamp(x, a, b):
    return max(a, min(x, b))

class VLANode(Node):
    def __init__(self):
        super().__init__('vla_node')

        self.srv_timeout = (
            self.declare_parameter('srv_timeout', 0.25, ParameterDescriptor(description='Timeout for waiting for xArm move service to be available (in seconds)'))
                .get_parameter_value().double_value
        )

        self.maxvel = (
            self.declare_parameter('maxvel', 50.0, ParameterDescriptor(description='Maximum robot linear velocity (in mm/s)'))
                .get_parameter_value().double_value
        )
        self.maxacc = (
            self.declare_parameter('maxacc', 500.0, ParameterDescriptor(description='Maximum robot linear acceleration (in mm/s^2)'))
                .get_parameter_value().double_value
        )

        self.maxtrans = (
            self.declare_parameter('maxtrans', 5.0, ParameterDescriptor(description='Maximum translation along each axis (in mm)'))
                .get_parameter_value().double_value
        )

        self.maxrot = (
            self.declare_parameter('maxrot', 5.0, ParameterDescriptor(description='Maximum rotation about each axis (in deg)'))
                .get_parameter_value().double_value
        )

        self.cb_group = ReentrantCallbackGroup()

        self.delta_sub = self.create_subscription(
            CartesianDelta, '/vla/output', # Cartesian deltas from VLA (e.g. OpenVLA)
            self.command_cb,
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, depth=1), # discard missed messages
            callback_group=self.cb_group
        )

        self.move_cli = self.create_client(
            MoveCartesian, '/xarm/set_position',
            callback_group=self.cb_group
        )

        self.gripper_cli = self.create_client(
            GripperMove, '/xarm/set_gripper_position',
            callback_group=self.cb_group
        )

        # enable arm
        rclpy.spin_until_future_complete(
            self,
            self.create_client(
                SetInt16ById, '/xarm/motion_enable'
            ).call_async(SetInt16ById.Request(id=8, data=1))
        )
        rclpy.spin_until_future_complete(
            self,
            self.create_client(
                SetInt16, '/xarm/set_mode'
            ).call_async(SetInt16.Request(data=0))
        )
        rclpy.spin_until_future_complete(
            self,
            self.create_client(
                SetInt16, '/xarm/set_state'
            ).call_async(SetInt16.Request(data=0))
        )

        # enable gripper
        rclpy.spin_until_future_complete(
            self,
            self.create_client(
                SetInt16, '/xarm/set_gripper_enable'
            ).call_async(SetInt16.Request(data=1))
        )

        self.get_logger().info('node started')

    def command_cb(self, msg: CartesianDelta):
        if not self.move_cli.wait_for_service(timeout_sec=self.srv_timeout):
            self.get_logger().error('timed out waiting for xArm move service - is the driver running?')
            return
        
        move_req = MoveCartesian.Request()
        move_req.wait = True # wait until completion before taking next command (TODO: maybe make the robot interruptible?)

        move_req.is_tool_coord = True
        move_req.relative = True

        move_req.speed = self.maxvel
        move_req.acc = self.maxacc
        
        rot = Rotation.from_quat([
            msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w
        ])
        rx, ry, rz = rot.as_euler('xyz', degrees=True).tolist() # get delta roll/pitch/yaw in degrees

        move_req.pose = [
            # translation (in mm)
            clamp(msg.transform.translation.x * 1000, -self.maxtrans, self.maxtrans),
            clamp(msg.transform.translation.y * 1000, -self.maxtrans, self.maxtrans),
            clamp(msg.transform.translation.z * 1000, -self.maxtrans, self.maxtrans),

            # rotation (in degrees)
            clamp(rx, -self.maxrot, self.maxrot),
            clamp(ry, -self.maxrot, self.maxrot),
            clamp(rz, -self.maxrot, self.maxrot)
        ]

        grip_req = GripperMove.Request()
        grip_req.pos = msg.gripper * 850 # 850 being the maximum pos

        self.get_logger().info(f'movement cmd: dx {move_req.pose[0]:.1f} dy {move_req.pose[1]:.1f} dz {move_req.pose[2]:.1f} drx {move_req.pose[3]:.1f} dry {move_req.pose[4]:.1f} drz {move_req.pose[5]:.1f} grip {grip_req.pos:.1f}')

        move_event = Event()
        grip_event = Event()

        t_start = time.time()

        move_future = self.move_cli.call_async(move_req)
        move_future.add_done_callback(lambda f, e=move_event: e.set())
        move_event.wait() # wait for completion

        grip_future = self.gripper_cli.call_async(grip_req)
        grip_future.add_done_callback(lambda f, e=grip_event: e.set())
        grip_event.wait()

        t_end = time.time()

        self.get_logger().info(f'movement executed in {(t_end - t_start):.4f} sec')

def main(args=None):
    rclpy.init(args=args)

    node = VLANode()

    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
