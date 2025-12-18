import rclpy
from rclpy.node import Node

# hack to call service from callback: https://gist.github.com/driftregion/14f6da05a71a57ef0804b68e17b06de5
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from threading import Event, Thread

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data

from rcl_interfaces.msg import ParameterDescriptor
from vla_msgs.msg import CartesianDelta
from xarm_msgs.srv import MoveCartesian, SetInt16, SetInt16ById, GripperMove # using SDK move function
from xarm_msgs.msg import RobotMsg
import time

from scipy.spatial.transform import Rotation

import numpy as np

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
            self.declare_parameter('maxrot', 0.08, ParameterDescriptor(description='Maximum rotation about each axis (in rad)'))
                .get_parameter_value().double_value
        )

        self.cb_group = ReentrantCallbackGroup()

        self.delta_sub = self.create_subscription(
            CartesianDelta, '/vla/output', # Cartesian deltas from VLA (e.g. OpenVLA)
            self.command_cb,
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, depth=1), # discard missed messages
            callback_group=self.cb_group
        )

        self.current_pose = None
        self.state_sub = self.create_subscription(
            RobotMsg, '/xarm/robot_states',
            self.state_cb,
            qos_profile_sensor_data,
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

        self.latest_cmd = None
        self.cmd_event = Event()

        self.worker_thread = Thread(target=self.worker_loop, daemon=True)
        self.worker_thread.start()

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

    def state_cb(self, msg: RobotMsg):
        self.current_pose = np.float64(msg.pose)
        self.get_logger().info(f'pose: x {self.current_pose[0]:.1f} y {self.current_pose[1]:.1f} z {self.current_pose[2]:.1f} dx {self.current_pose[3]:.1f} dy {self.current_pose[4]:.1f} dz {self.current_pose[5]:.1f}', throttle_duration_sec=1.0)

    def command_cb(self, msg: CartesianDelta):
        self.latest_cmd = msg
        self.cmd_event.set()

    def worker_loop(self):
        while rclpy.ok():
            self.cmd_event.wait()

            msg = self.latest_cmd
            self.cmd_event.clear()
            
            if self.current_pose is None:
                self.get_logger().error('robot pose has not been received yet')
                return
            
            if not self.move_cli.wait_for_service(timeout_sec=self.srv_timeout):
                self.get_logger().error('timed out waiting for xArm move service - is the driver running?')
                return
            
            move_req = MoveCartesian.Request()
            move_req.wait = True # wait until completion before taking next command (TODO: maybe make the robot interruptible?)

            move_req.is_tool_coord = True
            move_req.relative = False

            move_req.speed = self.maxvel
            move_req.acc = self.maxacc
            
            rot = Rotation.from_quat([
                msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w
            ])
            rx, ry, rz = rot.as_euler('xyz', degrees=False).tolist() # get delta roll/pitch/yaw in radians

            pose_offset = np.float64([
                # translation (in mm)
                clamp(msg.transform.translation.x * 1000, -self.maxtrans, self.maxtrans),
                clamp(msg.transform.translation.y * 1000, -self.maxtrans, self.maxtrans),
                clamp(msg.transform.translation.z * 1000, -self.maxtrans, self.maxtrans),

                # rotation (in degrees)
                clamp(rx, -self.maxrot, self.maxrot),
                clamp(ry, -self.maxrot, self.maxrot),
                clamp(rz, -self.maxrot, self.maxrot)
            ])

            move_req.pose = (self.current_pose + pose_offset).tolist()

            grip_req = GripperMove.Request()
            grip_req.pos = msg.gripper * 850 # 850 being the maximum pos

            self.get_logger().info(f'movement cmd: x {move_req.pose[0]:.1f} y {move_req.pose[1]:.1f} z {move_req.pose[2]:.1f} rx {move_req.pose[3]:.1f} ry {move_req.pose[4]:.1f} rz {move_req.pose[5]:.1f} grip {grip_req.pos:.1f}')

            move_event = Event()
            grip_event = Event()

            t_start = time.time()

            # move_future = self.move_cli.call_async(move_req)
            # move_future.add_done_callback(lambda f, e=move_event: e.set())
            # move_event.wait() # wait for completion
            self.move_cli.call(move_req)

            # grip_future = self.gripper_cli.call_async(grip_req)
            # grip_future.add_done_callback(lambda f, e=grip_event: e.set())
            # grip_event.wait()
            self.gripper_cli.call(grip_req)

            t_end = time.time()

            self.get_logger().info(f'movement executed in {(t_end - t_start):.4f} sec')

def main(args=None):
    rclpy.init(args=args)

    node = VLANode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
