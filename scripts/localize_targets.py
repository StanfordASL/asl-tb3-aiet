#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
from asl_tb3_msgs.msg import TurtleBotState
from asl_tb3_aiet.msg import TargetMarker
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, String
import math
import numpy as np
from scipy.spatial import ConvexHull
from typing import Dict
from task_base import Target

class TargetLocalizer(Node):
    def __init__(self):
        super().__init__('target_localizer_node')

        # Subscribers
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.state_sub = self.create_subscription(TurtleBotState, '/state', self.state_callback, 10)
        self.detector_class_sub = None

        # Publishers
        self.viz_marker_pub = self.create_publisher(Marker, '/marker_visualization', 10)
        self.target_marker_pub = self.create_publisher(TargetMarker, '/target_marker', 10)

        # Setup localizer parameters
        # Increasing the cluster_radius and search_fov should allow for more flexibility
        self.declare_parameter("cluster_radius", 0.3)           # radius to cluster LIDAR data (meters)
        self.declare_parameter("distance_buffer", 0.4)          # offset distance from location to ensure conflict-free (meters)
        self.declare_parameter("search_fov", 35)                # Angle of search for target (deg)

        # Add marker tracking and timestamp tracking
        self.marker_ids = {
            "stop sign": 0,
            "traffic light": 100  # Use different base IDs for different targets
        }
        self.target_database: Dict[str, Target] = {}
        self.latest_detection_time = None
        self.latest_detection_id = None
        self.processing_delay = 2.0  # 100ms delay to ensure we get fresh laser data
        self.last_detection_time = -np.inf
        self.prev_target_id = None

        self.imu_data = None
        self.scan_data = None
        self.state = None

    def imu_callback(self, msg):
        self.imu_data = msg
        self.start_detect_sub()

    def scan_callback(self, msg):
        self.scan_data = msg
        self.start_detect_sub()

    def state_callback(self, msg):
        self.state = msg
        self.start_detect_sub()
    
    def start_detect_sub(self):
        if self.imu_data != None and self.scan_data != None and self.state != None and self.detector_class_sub == None:
            self.detector_class_sub = self.create_subscription(String, '/detector_class', self.localize_targets_callback, 10)
    
    @property
    def cluster_radius(self) -> float:
        return self.get_parameter("cluster_radius").value

    @property
    def distance_buffer(self) -> float:
        return self.get_parameter("distance_buffer").value

    @property
    def search_fov(self) -> float:
        return self.get_parameter("search_fov").value

    def localize_targets_callback(self, msg: String):
        current_time = self.get_clock().now().nanoseconds        

        elapsed_time = current_time - self.last_detection_time
        # Extract Target ID from detector
        target_id = msg.data

        if self.prev_target_id is None:
            self.prev_target_id = target_id
        elif self.prev_target_id != target_id and elapsed_time <= self.processing_delay:
            self.get_logger().info("Skipping outdated detection.")
            self.prev_target_id = None
            return False

        # Extract yaw from IMU data
        orientation_q = self.imu_data.orientation
        w = orientation_q.w  # Using orientation_q.w as yaw angle

        # Compute the sector
        sector_angle = math.radians(self.search_fov / 2)
        sector_min = w - sector_angle
        sector_max = w + sector_angle

        # Get LaserScan data
        angle_min = self.scan_data.angle_min
        angle_increment = self.scan_data.angle_increment
        ranges = np.array(self.scan_data.ranges)
        num_ranges = len(ranges)

        # Create an array of angles corresponding to each index in ranges
        angles = angle_min + np.arange(num_ranges) * angle_increment

        # Shift angles so that sector_min aligns with 0
        angle_shift = sector_min
        angles_shifted = angles - angle_shift
        sector_min_shifted = 0.0
        sector_max_shifted = sector_max - sector_min

        # Wrap angles between 0 and 2*pi
        angles_shifted = np.mod(angles_shifted, 2 * math.pi)
        sector_max_shifted = np.mod(sector_max_shifted, 2 * math.pi)

        # Determine indices within the sector
        if sector_max_shifted >= sector_min_shifted:
            # Sector does not wrap around
            indices = np.where(
                (angles_shifted >= sector_min_shifted) & (angles_shifted <= sector_max_shifted)
            )[0]
        else:
            # Sector wraps around
            indices = np.where(
                (angles_shifted >= sector_min_shifted) | (angles_shifted <= sector_max_shifted)
            )[0]

        # Extract ranges and angles within the sector
        sector_ranges = ranges[indices]
        sector_angles = angles[indices]

        # Filter out invalid range readings (e.g., 'inf' or 'nan')
        valid_indices = np.isfinite(sector_ranges)
        sector_ranges = sector_ranges[valid_indices]
        sector_angles = sector_angles[valid_indices]
        indices = indices[valid_indices]

        if len(sector_ranges) == 0:
            self.get_logger().info("No valid ranges within the sector.")
            return

        # Find the minimum range within the sector
        min_range = np.min(sector_ranges)

        # Find all indices where the range is within cluster_radius of the minimum range
        close_indices_in_sector = np.where(sector_ranges <= min_range + self.cluster_radius)[0]
        close_indices = indices[close_indices_in_sector]

        # Group these indices into clusters where indices are consecutive
        clusters = []
        current_cluster = [close_indices[0]]
        for i in range(1, len(close_indices)):
            if close_indices[i] == close_indices[i - 1] + 1:
                current_cluster.append(close_indices[i])
            else:
                clusters.append(current_cluster)
                current_cluster = [close_indices[i]]
        clusters.append(current_cluster)  # Add the last cluster

        if not clusters:
            self.get_logger().info("No clusters found.")
            return

        # Find the cluster with the smallest range values
        cluster_min_ranges = []
        for cluster in clusters:
            cluster_ranges = ranges[cluster]
            cluster_min_range = np.min(cluster_ranges)
            cluster_min_ranges.append(cluster_min_range)

        # Select the cluster with the smallest minimum range
        min_cluster_index = np.argmin(cluster_min_ranges)
        min_cluster = clusters[min_cluster_index]
        min_cluster_ranges = ranges[min_cluster]
        min_cluster_angles = angles[min_cluster]

        # Compute target locations for each point in the selected cluster
        target_locations = []
        for range_value, angle_value in zip(min_cluster_ranges, min_cluster_angles):
            x_target, y_target = self.compute_target_location(self.state, range_value - self.distance_buffer, angle_value)
            target_locations.append((x_target, y_target))

        # DEBUGGING: Output the target locations
        # self.get_logger().info(f"Target Locations: {target_locations}")

        # Convert target_locations to numpy array
        points = np.array(target_locations)

        if len(points) >= 3:
            # Compute the minimum bounding rectangle
            bounding_box = self.minimum_bounding_rectangle(points)

            # Compute the centroid of the bounding box
            centroid = np.mean(bounding_box, axis=0)

            # Find the closest target location to the centroid
            distances = np.linalg.norm(points - centroid, axis=1)
            closest_point_idx = np.argmin(distances)
            
            # Get the angle associated with this point
            centroid_angle = min_cluster_angles[closest_point_idx] + self.state.theta
            
            # DEBUGGING: Log the centroid angle (for debugging)
            # self.get_logger().info(f"Centroid angle: {math.degrees(centroid_angle)} degrees")

            # Publish the centroid to /cmd_nav
            target_msg = self.publish_target(target_id, centroid, centroid_angle)

            # Publish visualization markers
            if target_msg.target_type not in self.target_database:
                self.target_database[target_msg.target_type] = Target(
                    x=target_msg.x,
                    y=target_msg.y,
                    theta=target_msg.theta,
                    confidence=target_msg.confidence
                )
                self.publish_viz_markers(target_id, bounding_box, centroid)
        else:
            self.get_logger().info("Not enough points to compute convex hull.")
        
        self.last_detection_time = self.get_clock().now().nanoseconds     
        return True

    def compute_target_location(self, state, range, theta_rel):
        # Calculate position relative to robot (in robot's frame)
        x_r = range * np.cos(theta_rel)
        y_r = range * np.sin(theta_rel)

        # Transform to global coordinates
        x_target = state.x + x_r * np.cos(state.theta) - y_r * np.sin(state.theta)
        y_target = state.y + x_r * np.sin(state.theta) + y_r * np.cos(state.theta)

        return x_target, y_target

    def minimum_bounding_rectangle(self, points):
        # Compute the convex hull
        hull_points = points[ConvexHull(points).vertices]

        # Calculate edge angles
        edges = np.diff(hull_points, axis=0)
        angles = np.arctan2(edges[:,1], edges[:,0])
        angles = np.abs(np.mod(angles, np.pi/2.0))
        angles = np.unique(angles)

        # Find rotation matrices
        rotations = np.array([[np.cos(angle), np.cos(angle - np.pi/2),
                               np.cos(angle + np.pi/2), np.cos(angle)] for angle in angles])
        rotations = rotations.reshape((-1, 2, 2))

        # Apply rotations to the hull
        rot_points = np.dot(rotations, hull_points.T)

        # Find the bounding rectangle for each rotation
        min_x = np.min(rot_points[:,0,:], axis=1)
        max_x = np.max(rot_points[:,0,:], axis=1)
        min_y = np.min(rot_points[:,1,:], axis=1)
        max_y = np.max(rot_points[:,1,:], axis=1)

        areas = (max_x - min_x) * (max_y - min_y)

        # Find the rotation with the minimum area
        best_idx = np.argmin(areas)
        best_rotation = rotations[best_idx]

        # Recover the coordinates of the bounding box
        x1 = best_rotation.T.dot([max_x[best_idx], min_y[best_idx]])
        x2 = best_rotation.T.dot([min_x[best_idx], min_y[best_idx]])
        x3 = best_rotation.T.dot([min_x[best_idx], max_y[best_idx]])
        x4 = best_rotation.T.dot([max_x[best_idx], max_y[best_idx]])
        rectangle = np.array([x1, x2, x3, x4])

        return rectangle

    def publish_viz_markers(self, target_id, bounding_box, centroid):
        base_id = self.marker_ids.get(target_id, 0)

        # Create a Marker for the bounding box
        box_marker = Marker()
        box_marker.header.frame_id = "map"  # Use appropriate frame
        box_marker.header.stamp = self.get_clock().now().to_msg()
        box_marker.ns = "bounding_box"
        box_marker.id = base_id
        box_marker.type = Marker.LINE_STRIP
        box_marker.action = Marker.ADD
        box_marker.pose.orientation.w = 1.0
        box_marker.scale.x = 0.05  # Line width
        box_marker.color.r = 1.0
        box_marker.color.g = 0.0
        box_marker.color.b = 0.0
        box_marker.color.a = 1.0
        box_marker.lifetime.sec = 0  # Make marker persistent

        # Add the corners of the bounding box
        for corner in np.vstack((bounding_box, bounding_box[0])):  # Close the loop
            p = Point()
            p.x = corner[0]
            p.y = corner[1]
            p.z = 0.1  # Assuming 2D plane
            box_marker.points.append(p)

        # Publish the bounding box marker
        self.viz_marker_pub.publish(box_marker)

        # Create a Marker for the centroid
        centroid_marker = Marker()
        centroid_marker.header.frame_id = "map"  # Use appropriate frame
        centroid_marker.header.stamp = self.get_clock().now().to_msg()
        centroid_marker.ns = "centroid"
        centroid_marker.text = target_id
        centroid_marker.id = base_id + 1
        centroid_marker.type = Marker.SPHERE
        centroid_marker.action = Marker.ADD
        centroid_marker.pose.position.x = centroid[0]
        centroid_marker.pose.position.y = centroid[1]
        centroid_marker.pose.position.z = 0.0  # Assuming 2D plane
        centroid_marker.scale.x = 0.1
        centroid_marker.scale.y = 0.1
        centroid_marker.scale.z = 0.1
        centroid_marker.color.r = 0.0
        centroid_marker.color.g = 1.0
        centroid_marker.color.b = 0.0
        centroid_marker.color.a = 1.0
        centroid_marker.lifetime.sec = 0  # Make marker persistent

        # Publish the centroid marker
        self.viz_marker_pub.publish(centroid_marker)

        # Create text marker for label
        text_marker = Marker()
        text_marker.header.frame_id = "map"
        text_marker.header.stamp = self.get_clock().now().to_msg()
        text_marker.ns = f"{target_id}_text"
        text_marker.id = base_id + 2
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = centroid[0]
        text_marker.pose.position.y = centroid[1]
        text_marker.pose.position.z = 0.3  # Place text above centroid
        text_marker.text = target_id
        text_marker.scale.z = 0.2  # Text height
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        text_marker.lifetime.sec = 0  # Make marker persistent

        # Publish the text marker
        self.viz_marker_pub.publish(text_marker)
    
    def publish_target(self, target_id, centroid, centroid_angle):
        centroid_msg = TargetMarker()
        centroid_msg.target_type = target_id
        centroid_msg.x = float(centroid[0])
        centroid_msg.y = float(centroid[1])
        centroid_msg.theta = centroid_angle  # Set to appropriate orientation if known
        centroid_msg.confidence = 1.0        # TODO: Hardcoded for now. Later, we'll want confidence from model

        # Publish the centroid message
        self.target_marker_pub.publish(centroid_msg)

        return centroid_msg


def main(args=None):
    rclpy.init(args=args)
    node = TargetLocalizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()