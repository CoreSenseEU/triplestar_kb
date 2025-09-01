import time

import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.node import Node
from ros2topic.api import get_msg_class


class QueryTimeSubscriber:
    """
    This class subscribes to a ros2 topic, inferring the message type, and keeps track of the latest message.
    The get method can be used to then retrieve the latest message at query time.
    """

    def __init__(
        self,
        node: Node | LifecycleNode,
        topic_name: str,
        msg_type=None,
        max_age_sec=2.0,
    ):
        self._latest_msg = None
        self._latest_time = None
        self._max_age_sec = max_age_sec
        self.node = node
        self.topic_name = topic_name
        self.logger = node.get_logger().get_child(f"query_time_subscriber:{topic_name}")

        self.wait_for_topic()
        msg_type = (
            get_msg_class(node, self.topic_name) if msg_type is None else msg_type
        )
        if msg_type is None:
            self.logger.error(
                f"Unable to determine message class for topic: {self.topic_name}"
            )
        node.create_subscription(msg_type, self.topic_name, self._callback, 10)
        self.logger.info(f"Subscribed to topic: {self.topic_name}")

    def wait_for_topic(
        self,
        timeout_sec: float = 30.0,
        poll_interval: float = 2,
    ) -> bool:
        start_time = time.time()
        while rclpy.ok():
            topics = [t[0] for t in self.node.get_topic_names_and_types()]
            if self.topic_name in topics:
                self.logger.info(f"Topic {self.topic_name} is now available.")
                return True
            if time.time() - start_time > timeout_sec:
                self.logger.error(f"Timeout waiting for topic {self.topic_name}.")
                return False
            self.logger.info(
                f"Waiting for topic {self.topic_name} to become available..."
            )
            time.sleep(poll_interval)
        return False

    def _callback(self, msg):
        self._latest_msg = msg
        self.logger.debug(f"feedback received: {self._latest_msg}")
        # Use header timestamp if available, else wall time
        if hasattr(msg, "header") and hasattr(msg.header, "stamp"):
            self._latest_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        else:
            self._latest_time = time.time()

    def get_latest(self):
        self.logger.debug("get_latest called")
        if self._latest_msg is not None and self._latest_time is not None:
            age = time.time() - self._latest_time
            if age < self._max_age_sec:
                return self._latest_msg
        return None
