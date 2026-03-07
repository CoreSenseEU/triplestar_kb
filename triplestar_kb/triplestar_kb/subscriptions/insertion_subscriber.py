from jinja2 import Environment
from rclpy.lifecycle import LifecycleNode
from rclpy.node import Node
from ros2topic.api import get_msg_class

from triplestar_kb.subscriptions.query_time_subscriber import wait_for_topic


class InsertionSubscriber:
    def __init__(self, node, config: dict, env: Environment, update_fn):
        self._node: Node | LifecycleNode = node
        self._logger = node.get_logger().get_child('InsertionSubscriber')
        self._topic = config['topic']
        self._update_fn = update_fn

        if not wait_for_topic(node, self._topic):
            raise RuntimeError(f'Topic {self._topic} not available')

        msg_type = get_msg_class(node, self._topic)
        if msg_type is None:
            raise RuntimeError(f'Unable to determine message class for {self._topic}')

        try:
            self._template = env.get_template(config['template'])
        except Exception as e:
            raise RuntimeError(f'Unable to load template {config["template"]}: {e}')

        self._sub = node.create_subscription(msg_type, self._topic, self._callback, 10)
        self._logger.info(f'Subscribed to {self._topic} with template {config["template"]}')

    def _callback(self, msg):
        try:
            query = self._template.render(msg=msg)
            self._update_fn(query)
            self._logger.debug(f'Insertion succeeded: {query[:100]}...')

        except Exception as e:
            self._logger.error(f'Insertion failed for {self._topic}: {e}')
