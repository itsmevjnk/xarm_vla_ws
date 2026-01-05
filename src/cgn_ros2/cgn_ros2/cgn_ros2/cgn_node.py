import rclpy
from rclpy.node import Node

from rcl_interfaces.msg import ParameterDescriptor

from cgn_msgs.srv import Infer
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge

from scipy.spatial.transform import Rotation

import torch
from torch_geometric.nn import fps
import numpy as np

from cgn_pytorch import CGN
import cgn_pytorch.util.config_utils as config_utils

import os
from ament_index_python.packages import get_package_share_directory

class ContactGraspNetNode(Node):
    def __init__(self):
        super().__init__('cgn_node')

        self.infer_srv = self.create_service(Infer, 'infer', self.infer_cb)

        self.model_path = (
            self.declare_parameter(
                'model_path', os.path.join(get_package_share_directory('cgn_ros2'), 'checkpoints'),
                ParameterDescriptor(description='Path to model (containing config.yaml and current.pth)')
            ).get_parameter_value().string_value
        )
        self.threshold = (
            self.declare_parameter(
                'threshold', 0.5,
                ParameterDescriptor(description='Confidence threshold for valid grasp selection')
            ).get_parameter_value().double_value
        )

        # load model
        if torch.cuda.is_available():
            torch.cuda.empty_cache() # not sure if this is needed
            self.device = torch.device('cuda')
        else:
            self.device = torch.device('cpu')
        config_dict = config_utils.load_config(self.model_path)
        self.model = CGN(config_dict, self.device).to(self.device)
        checkpoint = torch.load(os.path.join(self.model_path, 'current.pth'), map_location=self.device)
        self.model.load_state_dict(checkpoint['state_dict'])        
        self.model.eval()

        self.get_logger().info(f'model has been loaded to device {self.device}')

        self.bridge = CvBridge()

    def infer_cb(self, request: Infer.Request, response: Infer.Response):
        # depth to point cloud
        depth = self.bridge.imgmsg_to_cv2(request.depth, '16UC1')
        height, width = depth.shape
        self.get_logger().info(f'received depth image of size {width} x {height}')
        
        fx = request.info.k[0]; fy = request.info.k[4]
        cx = request.info.k[2]; cy = request.info.k[5]

        u, v = np.meshgrid(np.arange(width), np.arange(height))
        u = u.flatten(); v = v.flatten()
        
        z = depth.astype(np.float32).flatten() / 1000.0
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy

        points = np.stack((x, y, z), axis=-1)

        # extract valid points only
        valid_points_mask = z > 0
        points = points[valid_points_mask]
        mask = np.array(request.mask)[valid_points_mask]

        num_pts = points.shape[0]
        self.get_logger().info(f'input pointcloud has {num_pts} points')

        if num_pts > 20000:
            downsample = np.random.choice(np.arange(num_pts), 20000)
            points = points[downsample]
            mask = mask[downsample]

        # infer model
        points = torch.Tensor(points).to(dtype=torch.float32).to(self.device)
        batch = torch.zeros(points.shape[0]).to(dtype=torch.int64).to(self.device)
        idx = fps(points, batch, 2048 / points.shape[0]).to(self.device)
        
        mask = torch.as_tensor(mask, dtype=torch.float32, device=self.device)[idx]

        _, pred_grasps, confidence, _, _, _ = self.model(points[:, 3:], pos=points[:, :3], batch=batch, idx=idx)
        
        sig = torch.nn.Sigmoid()
        confidence = sig(confidence).reshape(-1, 1)

        pred_grasps = torch.flatten(pred_grasps, start_dim=0, end_dim=1).detach().cpu().numpy()
        confidence = (mask * confidence.reshape(-1)).detach().cpu().numpy()

        # filter by confidence threshold
        success_mask = (confidence > self.threshold).nonzero()[0]
        confidence = confidence[success_mask]
        pred_grasps = pred_grasps[success_mask]

        # sort by descending confidence
        sort_idx = np.argsort(-confidence)
        confidence = confidence[sort_idx]
        pred_grasps = pred_grasps[sort_idx]

        # construct responses
        response.grasps = []
        for grasp in pred_grasps:
            grasp_msg = PoseStamped()
            grasp_msg.header = request.depth.header
            grasp_msg.pose.position.x, grasp_msg.pose.position.y, grasp_msg.pose.position.z = grasp[:3, 3].tolist()
            grasp_msg.pose.orientation.x, grasp_msg.pose.orientation.y, grasp_msg.pose.orientation.z, grasp_msg.pose.orientation.w = Rotation.from_matrix(grasp[:3, :3]).as_quat().tolist()
            response.grasps.append(grasp_msg)
        response.confidence = confidence.tolist()

        return response

def main(args=None):
    rclpy.init(args=args)

    node = ContactGraspNetNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
