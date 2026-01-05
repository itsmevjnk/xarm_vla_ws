import rclpy
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from rclpy.node import Node

from rcl_interfaces.msg import ParameterDescriptor

import torch
from ultralytics import YOLO
from ultralytics.engine.results import Results

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import numpy as np
import matplotlib.cm as cm

import time

class YOLONode(Node):
    def __init__(self):
        super().__init__('yolo_node')

        self.model_name = (
            self.declare_parameter(
                'model', 'yolo11n-seg',
                ParameterDescriptor(description='Pretrained YOLO Instance Segmentation model to use')
            ).get_parameter_value().string_value
        )

        self.image_sub = (
            self.create_subscription(
                Image, 'image_in', self.image_cb,
                qos_profile_sensor_data
            )
        )

        self.mask_pub = self.create_publisher(Image, 'mask_out', qos_profile_system_default)
        self.image_pub = self.create_publisher(Image, 'image_out', qos_profile_system_default) # colourised image mask

        # load model
        if torch.cuda.is_available():
            torch.cuda.empty_cache() # not sure if this is needed
            self.device = torch.device('cuda')
        else:
            self.device = torch.device('cpu')
            
        self.model = YOLO(self.model_name).to(self.device)
        self.model.eval()

        self.get_logger().info(f'model has been loaded to device {self.device}')

        self.bridge = CvBridge()

        # Generate color palette: 80 distinct colors (besides black background)
        # Use matplotlib's nipy_spectral colormap for good color distinction
        colormap = cm.get_cmap('nipy_spectral')
        max_classes = 80
        self.colours = []
        for i in range(max_classes):
            rgba = colormap(i / max_classes)[:3]  # Get RGB (0-1 range)
            bgr = tuple(int(c * 255) for c in rgba[::-1])  # Convert to 0-255 BGR
            self.colours.append(bgr)

    def image_cb(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        height, width, _  = frame.shape

        t_start = time.time()
        result: Results = self.model(frame, verbose=False, show=False)[0]
        t_end = time.time()

        mask = np.ones((height, width), dtype=np.uint8) * 255 # 255 = background

        classifiers = result.boxes.cls.detach().cpu().numpy().astype(int).tolist()
        if len(classifiers) > 0:
            result_masks = result.masks.data.detach().cpu().numpy()
            for i, cls in enumerate(classifiers):
                mask[result_masks[i] > 0] = cls
        self.get_logger().info(f'inference completed in {(t_end - t_start):.4f} sec ({(1 / (t_end - t_start)):.1f} fps), got {len(classifiers)} classifier(s): {classifiers}', throttle_duration_sec=1.0)
        
        mask_msg = self.bridge.cv2_to_imgmsg(mask, '8UC1')
        mask_msg.header = msg.header
        self.mask_pub.publish(mask_msg)

        # create rgb image with colour set according to mask, 255 = black
        unique_ids = np.unique(mask)
        colourised = np.zeros_like(frame)
        for cid in unique_ids:
            if cid == 255:
                colourised[mask == 255] = (0, 0, 0)  # Black for background
            else:
                colour = self.colours[int(cid) % len(self.colours)]
                colourised[mask == cid] = colour

        colour_msg = self.bridge.cv2_to_imgmsg(colourised, 'bgr8')
        colour_msg.header = msg.header
        self.image_pub.publish(colour_msg)

def main(args=None):
    rclpy.init(args=args)

    node = YOLONode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
