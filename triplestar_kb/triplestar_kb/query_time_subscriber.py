import time
from typing import Any, Optional, Type

import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.node import Node
from ros2topic.api import get_msg_class


class QueryTimeSubscriber:
    _latest_msg: Optional[Any]
    _latest_time: Optional[float]
    _max_age_sec: float
    _node: Node | LifecycleNode
    _topic_name: str
    _msg_field_name: Optional[str]
    _logger: Any

    def __init__(
        self,
        node: Node | LifecycleNode,
        topic_name: str,
        msg_field_name: Optional[str] = None,
        msg_type: Optional[Type] = None,
        max_age_sec=2.0,
    ):
        self._node = node
        self._topic_name = topic_name
        self._msg_field_name = msg_field_name
        self._max_age_sec = max_age_sec
        self._latest_msg = None
        self._latest_time = None
        self._logger = node.get_logger().get_child(
            f"query_time_subscriber:{topic_name}"
        )

        if not self.wait_for_topic():
            raise RuntimeError(
                f"Failed to initialize subscriber for topic {self._topic_name}"
            )

        msg_type = (
            get_msg_class(node, self._topic_name) if msg_type is None else msg_type
        )
        if msg_type is None:
            self._logger.error(
                f"Unable to determine message class for topic: {self._topic_name}"
            )
            raise RuntimeError(
                f"Message type could not be determined for topic {self._topic_name}"
            )

        if self._msg_field_name is not None:
            if not hasattr(msg_type, self._msg_field_name):
                self._logger.error(
                    f"Message type {msg_type} does not have field {self._msg_field_name}"
                )
                raise AttributeError(
                    f"Field {self._msg_field_name} not found in message type {msg_type}"
                )
            else:
                self._logger.info(
                    f"Subscribing to topic: {self._topic_name} with message type: {msg_type} and field: {self._msg_field_name}"
                )

        node.create_subscription(msg_type, self._topic_name, self._callback, 10)

        self._logger.info(f"Subscribed to topic: {self._topic_name}")

    def wait_for_topic(
        self,
        timeout_sec: float = 30.0,
        poll_interval: float = 2,
    ) -> bool:
        start_time = time.time()
        while rclpy.ok():
            topics = [t[0] for t in self._node.get_topic_names_and_types()]
            if self._topic_name in topics:
                self._logger.info(f"Topic {self._topic_name} is now available.")
                return True
            if time.time() - start_time > timeout_sec:
                self._logger.error(f"Timeout waiting for topic {self._topic_name}.")
                return False
            self._logger.info(
                f"Waiting for topic {self._topic_name} to become available..."
            )
            time.sleep(poll_interval)
        return False

    def _callback(self, msg):
        self._latest_msg = msg
        self._logger.debug(f"feedback received: {self._latest_msg}")
        # Use header timestamp if available, else wall time
        if hasattr(msg, "header") and hasattr(msg.header, "stamp"):
            self._latest_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        else:
            self._latest_time = time.time()

    def get_latest(self):
        self._logger.debug("get_latest called")
        if not self._latest_msg or not self._latest_time:
            return None

        if (time.time() - self._latest_time) < self._max_age_sec:
            return (
                getattr(self._latest_msg, self._msg_field_name, self._latest_msg)
                if self._msg_field_name
                else self._latest_msg
            )

        return None
