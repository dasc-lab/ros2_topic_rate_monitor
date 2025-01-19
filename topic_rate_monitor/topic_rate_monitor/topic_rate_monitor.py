import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from time import time
import yaml
from prettytable import PrettyTable
from rosidl_runtime_py.utilities import get_message
from rclpy.qos import qos_profile_sensor_data
import os
from collections import deque
import numpy as np

from topic_rate_monitor_interfaces.msg import TopicRate, TopicRateArray


class TopicRateMonitor(Node):
    def __init__(self):
        super().__init__("topic_rate_monitor")

        # Get the config path
        self.declare_parameter("topics", "topics.yaml")
        config_file = self.get_parameter("topics").get_parameter_value().string_value

        self.declare_parameter("clear_screen", True)
        self.clear_screen = (
            self.get_parameter("clear_screen").get_parameter_value().bool_value
        )

        # Load configuration
        self.topics = self.load_config(config_file)

        # Variables to track message timestamps
        self.topic_info = {
            topic["name"]: {
                "last_time": np.nan,
                "rate_buffer": deque(maxlen=topic.get("window", 10)),
                "rate": np.nan,
                "jitter": np.nan,
                "status": "N/A",
            }
            for topic in self.topics
        }

        # variable to track total processing time
        self.total_proc_time = 0

        ## Create subscriptions
        for topic in self.topics:

            topic_name = topic["name"]

            self.get_logger().info(f"creating subscription for {topic_name}")

            self.create_subscription(
                get_message(topic["type"]),
                topic_name,
                self.create_callback(topic_name),
                qos_profile_sensor_data,
                raw=True,
            )

        # create publisher
        self.publisher = self.create_publisher(TopicRateArray, "~/topic_rates", 10)

        # Timer to periodically display rates
        self.print_timer = self.create_timer(1.0, self.print_rates)
        self.publish_timer = self.create_timer(1.0, self.publish_msg)

        # print it once to clear the screen
        self.print_rates()

    def load_config(self, config_file):
        self.get_logger().info(f"Reading config file: {config_file}")
        try:
            with open(config_file, "r") as f:
                config = yaml.safe_load(f)
            return config["topics"]
        except Exception as e:
            self.get_logger().error(f"Failed to load configuration file: {e}")
            self.destroy_node()
            raise SystemExit

    def create_callback(self, topic_name):

        def callback(self, msg):
            self.message_callback(topic_name)

        return lambda msg: callback(self, msg)

    def message_callback(self, topic_name):
        current_time = time()
        self.get_logger().debug(f"Got message from {topic_name} at time {current_time}")
        topic_data = self.topic_info[topic_name]
        if not np.isnan(topic_data["last_time"]):

            # Calculate the time difference and rate
            dt = current_time - topic_data["last_time"]
            rate = 1.0 / dt

            # push the new value to the window
            topic_data["rate_buffer"].append(rate)

            # update the rate
            topic_data["rate"] = self.get_rate(topic_data["rate_buffer"])

            # update the jitter
            topic_data["jitter"] = self.get_jitter(topic_data["rate_buffer"])

        topic_data["last_time"] = current_time

        # also keep track of the total processing time
        proc_time = time() - current_time
        self.total_proc_time += proc_time

    def get_rate(self, buffer):
        return np.mean(buffer)

    def get_jitter(self, buffer):
        return np.std(np.array(buffer))

    def get_status_string(self, expected_rate, observed_rate, tolerance, time_since):
        if np.isnan(expected_rate) or expected_rate <= 0:
            status = "WARNING: no expected rate"
            return status

        if time_since > 3 / expected_rate:
            status = "WARNING: no new msgs"
            return status

        if abs((observed_rate - expected_rate) / expected_rate) > tolerance:
            status = "WARNING: stale msgs"
            return status

        status = "OK"
        return status

    def print_rates(self):

        # Create a pretty table to display rates
        table = PrettyTable()
        table.field_names = [
            "Topic Name",
            "Expected Rate (Hz)",
            "Observed Rate (Hz)",
            "Jitter (Hz)",
            "Time since last (s)",
            "Status",
        ]

        for topic in self.topics:
            name = topic["name"]
            expected_rate = topic.get("expected_rate", 0)
            tolerance = topic.get("tolerance", 0.1)
            observed_rate = self.topic_info[name]["rate"]
            jitter = self.topic_info[name]["jitter"]
            time_since_last = time() - self.topic_info[name].get("last_time", np.nan)
            status = self.get_status_string(
                expected_rate, observed_rate, tolerance, time_since_last
            )

            table.add_row(
                [
                    name,
                    (
                        f"{expected_rate:.2f} +- {tolerance * 100:2.0f}%"
                        if expected_rate > 0
                        else "N/A"
                    ),
                    f"{observed_rate:.2f}" if observed_rate else "N/A",
                    f"{jitter:.2f}",
                    f"{time_since_last:3.2f}",
                    status,
                ]
            )

        if self.clear_screen:
            os.system("clear")
        self.get_logger().info(
            f"\n{table}\nTotal Processing Time: {self.total_proc_time * 1e3:4.3f} ms"
        )
        self.total_proc_time = 0

    def publish_msg(self):

        if self.publisher.get_subscription_count() == 0:
            return

        # create the TopicRatesArray message
        array_msg = TopicRateArray()
        for topic in self.topics:

            name = topic["name"]
            expected_rate = topic.get("expected_rate", 0)
            observed_rate = self.topic_info[name]["rate"]
            tolerance = topic.get("tolerance", 0.1)
            jitter = self.topic_info[name]["jitter"]
            time_since_last = time() - self.topic_info[name].get("last_time", np.nan)
            status = self.get_status_string(
                expected_rate, observed_rate, tolerance, time_since_last
            )

            # create a TopicRate message
            msg = TopicRate()
            msg.topic_name = name
            msg.expected_rate = expected_rate
            msg.observed_rate = observed_rate
            msg.tolerance = tolerance
            msg.jitter = jitter
            msg.time_since_last_msg = time_since_last
            msg.status = status

            array_msg.topics.append(msg)

        # publish
        self.publisher.publish(array_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TopicRateMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
