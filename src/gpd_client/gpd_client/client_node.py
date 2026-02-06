import rclpy
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from rclpy.node import Node

import numpy as np
from scipy.spatial.transform import Rotation
import base64

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped

from tf2_ros import TransformBroadcaster

import requests

class GPDClient(Node):
    def __init__(self):
        super().__init__('gpd_client')
        self.cloud_sub = self.create_subscription(PointCloud2, 'cloud', self.cloud_cb, qos_profile_sensor_data)
        self.pose_pub = self.create_publisher(TransformStamped, 'grasp_pose', qos_profile_system_default)
        self.tf_broadcaster = TransformBroadcaster(self)
    
    def cloud_cb(self, data: PointCloud2):
        # serialise PointCloud2 to JSON
        payload = {
            'cloud': {
                'header': {
                    'frame_id': data.header.frame_id,
                },
                'height': data.height,
                'width': data.width,
                'point_step': data.point_step,
                'row_step': data.row_step,
                'is_dense': data.is_dense,
                'is_bigendian': data.is_bigendian,
                'fields': [
                    {
                        'name': f.name,
                        'offset': f.offset,
                        'datatype': f.datatype,
                        'count': f.count,
                    }
                    for f in data.fields
                ],
                'data': base64.b64encode(data.data.tobytes()).decode('utf-8')
            }
        }

        try:
            response = requests.post('http://127.0.0.1:8000/detect_grasps', json=payload, timeout=15.0)
            response.raise_for_status()
            result = response.json()

            if not result['grasps']:
                self.get_logger().warn('no grasps detected')
                return
            
            self.publish_grasp_tf(result['grasps'][0], data.header.frame_id)
        except Exception as e:
            self.get_logger().error(f'HTTP request failed: {e}')
    
    def publish_grasp_tf(self, grasp: dict, frame_id: str):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = frame_id
        t.child_frame_id = 'grasp_pose'

        t.transform.translation.x = grasp['position']['x']
        t.transform.translation.y = grasp['position']['y']
        t.transform.translation.z = grasp['position']['z']

        rot_matrix = np.zeros((3, 3))
        rot_matrix[0,1] = -grasp['binormal']['x'] # hand closing direction
        rot_matrix[1,1] = -grasp['binormal']['y']
        rot_matrix[2,1] = -grasp['binormal']['z']
        rot_matrix[0,0] = grasp['axis']['x'] # hand axis
        rot_matrix[1,0] = grasp['axis']['y']
        rot_matrix[2,0] = grasp['axis']['z']
        rot_matrix[0,2] = grasp['approach']['x'] # approach direction
        rot_matrix[1,2] = grasp['approach']['y']
        rot_matrix[2,2] = grasp['approach']['z']
        rot_quat = Rotation.from_matrix(rot_matrix).as_quat().tolist()
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = rot_quat

        self.pose_pub.publish(t)
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info(f'published grasp with score {grasp["score"]}')

def main(args=None):
    rclpy.init(args=args)
    node = GPDClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()