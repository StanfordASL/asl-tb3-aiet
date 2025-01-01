import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, InteractiveMarker, InteractiveMarkerControl
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
import math


class LaserScanIndexInteractive(Node):
    def __init__(self):
        super().__init__('laser_scan_index_interactive')

        # Subscriber for LaserScan data
        self.scan_subscriber = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # MarkerArray publisher for displaying text indices
        self.marker_pub = self.create_publisher(Marker, '/scan_index_markers', 10)

        # Interactive marker server
        self.interactive_marker_server = InteractiveMarkerServer(self, 'interactive_scan_indices')

    def scan_callback(self, msg):
        # Clear existing markers
        self.interactive_marker_server.clear()

        for i, distance in enumerate(msg.ranges):
            if math.isinf(distance) or math.isnan(distance):
                continue

            # Calculate the (x, y) position of the point
            angle = msg.angle_min + i * msg.angle_increment
            x = distance * math.cos(angle)
            y = distance * math.sin(angle)

            # Publish a text marker for the index
            self.publish_text_marker(i, x, y, msg.header)

            # Add an interactive marker
            self.create_interactive_marker(i, x, y, msg.header)

        # Apply changes to the interactive marker server
        self.interactive_marker_server.applyChanges()

    def publish_text_marker(self, index, x, y, header):
        # Create a text marker for the index
        marker = Marker()
        marker.header = header
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.2  # Slightly above ground
        marker.scale.z = 0.2  # Size of the text
        marker.color.a = 1.0  # Opacity
        marker.color.r = 1.0  # Red color
        marker.color.g = 1.0  # Green color
        marker.color.b = 1.0  # Blue color
        marker.text = str(index)
        marker.id = index

        self.marker_pub.publish(marker)

    def create_interactive_marker(self, index, x, y, header):
        # Create an interactive marker
        int_marker = InteractiveMarker()
        int_marker.header = header
        int_marker.name = f"marker_{index}"
        int_marker.description = f"Index: {index}"
        int_marker.pose.position.x = x
        int_marker.pose.position.y = y
        int_marker.pose.position.z = 0.1  # Slightly above the ground

        # Add a control that shows the text
        control = InteractiveMarkerControl()
        control.interaction_mode = InteractiveMarkerControl.BUTTON
        control.always_visible = True

        # Add a visual marker (sphere) for the interactive marker
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0  # Opacity
        marker.color.r = 0.0  # Red
        marker.color.g = 1.0  # Green
        marker.color.b = 0.0  # Blue
        control.markers.append(marker)

        # Add the control to the interactive marker
        int_marker.controls.append(control)

        # Insert the interactive marker into the server
        self.interactive_marker_server.insert(int_marker)

        # Set the callback for this marker
        self.interactive_marker_server.setCallback(int_marker.name, self.process_feedback)

    def process_feedback(self, feedback):
        # Print feedback when the marker is clicked
        self.get_logger().info(f"Clicked on marker with description: {feedback.marker_name}")


def main():
    rclpy.init()
    node = LaserScanIndexInteractive()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
