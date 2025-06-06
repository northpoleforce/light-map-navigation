# Copyright (c) 2022 Hamburg Bit-Bots
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import cv2
from cv_bridge import CvBridge
from ipm_library.exceptions import CameraInfoNotSetException
from ipm_library.ipm import IPM
from ipm_library.utils import Plane, create_horizontal_plane
import numpy as np
from numpy.lib import recfunctions as rfn
import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from sensor_msgs_py.point_cloud2 import create_cloud
from std_msgs.msg import Header
import tf2_ros as tf2

cv_bridge = CvBridge()


class IPMImageNode(Node):

    def __init__(self) -> None:
        super().__init__('ipm_image_node')
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('output_frame', 'base_footprint'),
                ('type', 'mask'),
                ('scale', 1.0),
                ('use_distortion', False),
                ('plane_coefficients', [0.0, 0.0, 1.0, 0.230]),
                ('border_size', 5),
                ('kernel_size', 3),
                ('camera_info_topic', '/camera_sensor/camera_info'),
                ('point_cloud_topic', 'ipm_projected_point_cloud'),
                ('image_input_topic', '/grounded_sam2/mask'),
                ('cache_time', 30.0)
            ]
        )

        self.tf_buffer = tf2.Buffer(cache_time=Duration(seconds=self.get_parameter('cache_time').value))
        self.tf_listener = tf2.TransformListener(self.tf_buffer, self)
        self.ipm = IPM(self.tf_buffer, distortion=self.get_parameter('use_distortion').value)
        
        self.result_publisher = self.create_publisher(
            PointCloud2, 
            self.get_parameter('point_cloud_topic').value, 
            1
        )
        self.create_subscription(
            CameraInfo, 
            self.get_parameter('camera_info_topic').value,
            self.ipm.set_camera_info, 
            1
        )
        self.create_subscription(
            Image, 
            self.get_parameter('image_input_topic').value,
            self.map_message, 
            1
        )

    def _detect_edges(self, image: np.ndarray) -> np.ndarray:
        kernel_size = self.get_parameter('kernel_size').value
        border_size = self.get_parameter('border_size').value
        
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        edges = cv2.morphologyEx(image, cv2.MORPH_GRADIENT, kernel)
        
        h, w = edges.shape
        border_mask = np.ones((h, w), dtype=np.uint8)
        border_mask[0:border_size, :] = 0
        border_mask[-border_size:, :] = 0
        
        return edges * border_mask

    def _create_field_plane(self) -> Plane:
        field = Plane()
        coefficients = self.get_parameter('plane_coefficients').value
        field.coef = coefficients
        return field

    def map_message(self, msg: Image) -> None:
        scale = self.get_parameter('scale').value
        output_frame = self.get_parameter('output_frame').value
        field = self._create_field_plane()

        image = cv2.resize(
            cv_bridge.imgmsg_to_cv2(msg),
            (0, 0), fx=scale, fy=scale, 
            interpolation=cv2.INTER_NEAREST
        )

        image_type = self.get_parameter('type').value
        if image_type == 'mask':
            edges = self._detect_edges(image)
            point_idx_tuple = np.where(edges != 0)
        elif image_type == 'rgb_image':
            X, Y = np.meshgrid(np.arange(image.shape[1]), np.arange(image.shape[0]))
            point_idx_tuple = (Y.ravel(), X.ravel())
        else:
            self.get_logger().error(f"Unsupported image type '{image_type}'!")
            return

        point_idx_array = np.empty((point_idx_tuple[0].shape[0], 2))
        point_idx_array[:, 0] = point_idx_tuple[1] / scale
        point_idx_array[:, 1] = point_idx_tuple[0] / scale

        try:
            points_on_plane = self.ipm.map_points(
                field,
                point_idx_array,
                msg.header.stamp,
                plane_frame_id=output_frame,
                output_frame_id=output_frame
            )[1]
        except CameraInfoNotSetException:
            self.get_logger().warn('No camera info received yet!', throttle_duration_sec=5)
            return
        except (tf2.LookupException, tf2.ConnectivityException, tf2.ExtrapolationException) as e:
            self.get_logger().warn(f'Transform error: {e}', throttle_duration_sec=5)
            return

        # Define fields of the point cloud
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)]

        # Add rgb data to pointcloud
        if image_type == 'rgb_image':
            # Add additional field for rgb values
            fields.append(PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1))
            # Add unused alpha channel,
            # because the casting of 4 x uint8 -> uint32 is easier with 4 channels
            rgba = cv2.cvtColor(image, cv2.COLOR_RGB2RGBA)
            # Recast 4 uint8 channels to one uint32 per pixel
            pixel_values = np.ndarray(
                shape=(image.shape[0] * image.shape[1], ),
                dtype=np.uint32,
                buffer=memoryview(rgba))
            # Create dtype for structured pointcloud numpy array
            new_dtype = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<u4')])
            # Allocate new array with said dtype that has space
            # for the float coordinates and color values
            combined_points = np.empty(len(pixel_values), dtype=new_dtype)
            # Copy xyz coordinates into new array
            combined_points[['x', 'y', 'z']] = rfn.unstructured_to_structured(
                points_on_plane,
                names=('x', 'y', 'z')
            )
            # Copy rgb values (that are stored in a single uint32)
            combined_points['rgb'] = pixel_values
            # Use common name for final point cloud
            points_on_plane = combined_points

        # Build pointcloud
        pc = create_cloud(
            Header(
                stamp=msg.header.stamp,
                frame_id=output_frame
            ),
            fields,
            points_on_plane)

        self.result_publisher.publish(pc)


def main(args=None):
    rclpy.init(args=args)
    node = IPMImageNode()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    ex.spin()
    node.destroy_node()
    rclpy.shutdown()
