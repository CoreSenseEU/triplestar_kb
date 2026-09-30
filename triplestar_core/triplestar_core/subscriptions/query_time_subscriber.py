import time
from typing import Any

from geometry_msgs.msg import Vector3
from rclpy.lifecycle import LifecycleNode
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros


class BaseLatestSubscriber:
    def __init__(
        self,
        node: Node | LifecycleNode,
        logger,
        name: str,
        max_age_sec: float = 2.0,
    ) -> None:
        self._node = node
        self._max_age_sec = max_age_sec
        self._logger = logger.get_child(name)

    def get_latest(self, *args, **kwargs) -> Any | None:
        raise NotImplementedError('get_latest must be implemented by subclasses')


class TopicLatestSubscriber(BaseLatestSubscriber):
    def __init__(
        self,
        node: Node | LifecycleNode,
        logger,
        topic: str,
        msg_type,
        callback_group,
        max_age_sec: float = 2.0,
        target_msg_field: str | None = None,
    ):
        super().__init__(node, logger, topic.strip('/').replace('/', '.'), max_age_sec)
        self._topic = topic
        self._target_msg_field = target_msg_field
        self._latest_msg = None
        self._latest_time = None

        if self._target_msg_field:
            try:
                self._resolve_target_field(msg_type())
            except (AttributeError, TypeError) as e:
                raise RuntimeError(
                    f'Message type {msg_type} does not have field path {self._target_msg_field}'
                ) from e

        self._subscription = self._node.create_subscription(
            msg_type,
            self._topic,
            self._callback,
            10,
            callback_group=callback_group,
        )
        self._logger.info(f'Subscribed to {self._topic}')

    def destroy(self) -> None:
        self._node.destroy_subscription(self._subscription)

    def _callback(self, msg):
        self._latest_msg = msg
        self._latest_time = (
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if hasattr(msg, 'header') and hasattr(msg.header, 'stamp')
            else self._node.get_clock().now().nanoseconds * 1e-9
        )

    def _resolve_target_field(self, msg):
        assert self._target_msg_field is not None
        value = msg
        for field_name in self._target_msg_field.split('.'):
            value = getattr(value, field_name)
        return value

    def get_latest(self, *args, **kwargs):
        if not self._latest_msg or not self._latest_time:
            return None
        if (time.time() - self._latest_time) >= self._max_age_sec:
            return None
        if self._target_msg_field:
            return self._resolve_target_field(self._latest_msg)
        return self._latest_msg


class TransformPositionLookup:
    """Look up fresh frame positions from a TF buffer without blocking."""

    def __init__(self, buffer: tf2_ros.Buffer, clock, logger, max_age_sec: float = 2.0):
        self._buffer = buffer
        self._clock = clock
        self._logger = logger
        self._max_age_nanoseconds = int(max_age_sec * 1e9)

    def get_position(self, frame: str, reference_frame: str) -> Vector3 | None:
        """Return *frame*'s origin in *reference_frame*, or ``None`` if unavailable."""
        try:
            # Omitting a timeout makes this an immediate lookup. A blocking lookup from
            # a query callback could prevent this node's executor from receiving TF.
            transform = self._buffer.lookup_transform(reference_frame, frame, Time())
        except Exception as e:  # noqa: BLE001
            self._logger.warning(
                f'TF lookup failed for {frame} in reference frame {reference_frame}: {e}'
            )
            return None

        stamp = transform.header.stamp
        stamp_nanoseconds = stamp.sec * 1_000_000_000 + stamp.nanosec
        age_nanoseconds = self._clock.now().nanoseconds - stamp_nanoseconds
        if age_nanoseconds >= self._max_age_nanoseconds:
            self._logger.warning(
                f'TF lookup for {frame} in reference frame {reference_frame} is stale'
            )
            return None

        return transform.transform.translation


class TransformLatestSubscriber(BaseLatestSubscriber):
    def __init__(
        self,
        node: Node | LifecycleNode,
        logger,
        from_frame: str,
        to_frame: str,
        buffer: tf2_ros.Buffer,
        listener: tf2_ros.TransformListener,
        max_age_sec: float = 2.0,
    ):
        super().__init__(node, logger, f'{from_frame}_to_{to_frame}', max_age_sec)
        self._from_frame = from_frame
        self._to_frame = to_frame
        self._buffer = buffer
        self._listener = listener
        self._lookup = TransformPositionLookup(
            buffer=self._buffer,
            clock=self._node.get_clock(),
            logger=self._logger,
            max_age_sec=self._max_age_sec,
        )

    def get_latest(self) -> Vector3 | None:
        return self._lookup.get_position(self._from_frame, self._to_frame)
