import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import matplotlib.pyplot as plt

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped

# -------- CONFIG --------
BAG_PATH = "/home/aa274/asl_tb3_aiet/rosbag2_2026_01_26-15_54_32"   # <-- path to your bag directory
CMD_VEL_TOPIC = "/cmd_vel"
POSE_TOPIC = "/pose"
# ------------------------

def read_bag(bag_path):
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(
        uri=bag_path,
        storage_id="sqlite3"
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr"
    )
    reader.open(storage_options, converter_options)

    topic_types = {
        topic.name: topic.type
        for topic in reader.get_all_topics_and_types()
    }

    cmd_vel_times = []
    lin_x = []
    ang_z = []

    pose_x = []
    pose_y = []

    while reader.has_next():
        topic, data, t = reader.read_next()
        msg_type = get_message(topic_types[topic])
        msg = deserialize_message(data, msg_type)

        time_sec = t * 1e-9

        if topic == CMD_VEL_TOPIC:
            cmd_vel_times.append(time_sec)
            lin_x.append(msg.linear.x)
            ang_z.append(msg.angular.z)

        elif topic == POSE_TOPIC:
            pose_x.append(msg.position.x)
            pose_y.append(msg.position.y)

    return cmd_vel_times, lin_x, ang_z, pose_x, pose_y


def plot_data(times, lin_x, ang_z, pose_x, pose_y):
    plt.figure(figsize=(12, 5))

    # Velocity plot
    plt.subplot(1, 2, 1)
    plt.plot(times, lin_x, label="Linear X (m/s)")
    plt.plot(times, ang_z, label="Angular Z (rad/s)")
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity")
    plt.title("/cmd_vel")
    plt.legend()
    plt.grid(True)

    # Trajectory plot
    plt.subplot(1, 2, 2)
    plt.plot(pose_x, pose_y, marker=".")
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.title("Robot Trajectory (/pose)")
    plt.axis("equal")
    plt.grid(True)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    times, lin_x, ang_z, pose_x, pose_y = read_bag(BAG_PATH)
    plot_data(times, lin_x, ang_z, pose_x, pose_y)
