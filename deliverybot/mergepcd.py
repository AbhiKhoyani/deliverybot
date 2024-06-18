import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
import numpy as np
import pandas as pd
from pyntcloud import PyntCloud

class PointCloudMerger(Node):
    def __init__(self):
        super().__init__('point_cloud_merger')
        self.subscription1 = self.create_subscription(
            PointCloud2,
            '/camera_left/points',
            self.point_cloud_callback_left,
            10)
        self.subscription2 = self.create_subscription(
            PointCloud2,
            '/camera_right/points',
            self.point_cloud_callback_right,
            10)
        self.publisher = self.create_publisher(PointCloud2, '/base/points', 10)
        self.pointcloud_left = None
        self.pointcloud_right = None

    def point_cloud_callback_left(self, msg):
        self.pointcloud_left = self.convert_ros_to_pyntcloud(msg)
        self.process_point_clouds()

    def point_cloud_callback_right(self, msg):
        self.pointcloud_right = self.convert_ros_to_pyntcloud(msg)
        self.process_point_clouds()

    def convert_ros_to_pyntcloud(self, ros_cloud):
        points_list = []

        for data in pc2.read_points(ros_cloud, skip_nans=True):
            points_list.append([data[0], data[1], data[2]])

        # Convert to PyntCloud
        df = pd.DataFrame(points_list, columns=['x', 'y', 'z'])
        pyntcloud = PyntCloud(df)

        return pyntcloud

    def apply_transformation(self, cloud, transformation_matrix):
        points = cloud.points[['x', 'y', 'z']].values
        transformed_points = np.dot(points, transformation_matrix[:3, :3].T) + transformation_matrix[:3, 3]
        transformed_df = pd.DataFrame(transformed_points, columns=['x', 'y', 'z'])
        transformed_cloud = PyntCloud(transformed_df)

        return transformed_cloud

    def process_point_clouds(self):
        if self.pointcloud_left is not None and self.pointcloud_right is not None:
            # Define transformation matrices (example identity matrices here)
            tf_left2base = np.array([[-0.669, 0.001,  0.743,  0.039],
                                    [-0.743, -0.001, -0.669,  0.022],
                                    [0.000, -1.000,  0.001, 0.165],
                                    [0.000,  0.000,  0.000,  1.000]])
            
            tf_right2base = np.array([[0.669, 0.001,  0.743,  0.040],
                                    [-0.743, 0.001, 0.669,  -0.022],
                                    [0.000, -1.000,  0.001, 0.165],
                                    [0.000,  0.000,  0.000,  1.000]])


            # Apply transformations
            transformed_cloud_left = self.apply_transformation(self.pointcloud_left, tf_left2base)
            transformed_cloud_right = self.apply_transformation(self.pointcloud_right, tf_right2base)

            # Merge point clouds
            merged_cloud = self.merge_point_clouds(transformed_cloud_left, transformed_cloud_right)

            # Convert to ROS message and publish
            ros_merged_cloud = self.convert_pyntcloud_to_ros(merged_cloud)
            self.publisher.publish(ros_merged_cloud)

    def merge_point_clouds(self, cloud1, cloud2):
        merged_df = pd.concat([cloud1.points, cloud2.points], ignore_index=True)
        merged_cloud = PyntCloud(merged_df)
        return merged_cloud

    def convert_pyntcloud_to_ros(self, pyntcloud):
        points = pyntcloud.points[['x', 'y', 'z']].values
        ros_cloud = PointCloud2()
        ros_cloud.header.stamp = self.get_clock().now().to_msg()
        ros_cloud.header.frame_id = 'base_link'
        ros_cloud.height = 1
        ros_cloud.width = points.shape[0]
        ros_cloud.fields = [
            pc2.PointField(name='x', offset=0, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='y', offset=4, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='z', offset=8, datatype=pc2.PointField.FLOAT32, count=1)
        ]
        ros_cloud.is_bigendian = False
        ros_cloud.point_step = 12
        ros_cloud.row_step = ros_cloud.point_step * points.shape[0]
        ros_cloud.is_dense = True
        ros_cloud.data = np.asarray(points, np.float32).tobytes()

        return ros_cloud

def main(args=None):
    rclpy.init(args=args)
    point_cloud_merger = PointCloudMerger()
    rclpy.spin(point_cloud_merger)
    point_cloud_merger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
