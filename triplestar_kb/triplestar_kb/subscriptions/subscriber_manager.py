import tf2_ros
from jinja2 import Environment, FileSystemLoader
from pyoxigraph import NamedNode

from triplestar_kb.kb_interface import TriplestarKBInterface
from triplestar_kb.msg_to_rdf import ros_msg_to_literal
from triplestar_kb.subscriptions.insertion_subscriber import InsertionSubscriber
from triplestar_kb.subscriptions.query_time_subscriber import (
    QueryTimeTFSubscriber,
    QueryTimeTopicSubscriber,
)

EX = 'http://example.org/'


def _rdf_filter(value) -> str:
    literal = ros_msg_to_literal(value)
    return str(literal) if literal is not None else repr(value)


class SubscriberManager:
    def __init__(self, node, config: dict, kb: TriplestarKBInterface):
        self.node = node
        self.logger = node.get_logger().get_child('subscriber_manager')

        self._buffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._buffer, node)

        self.topic_query_subs: dict[str, QueryTimeTopicSubscriber] = {}
        self.tf_query_subs: dict[str, QueryTimeTFSubscriber] = {}
        self.insertion_subs: dict[str, InsertionSubscriber] = {}

        templates_dir = node.get_parameter('templates_dir').value
        env = Environment(loader=FileSystemLoader(templates_dir))
        env.filters['rdf'] = _rdf_filter

        self._load_config(config, kb, env)

        # register the query-time data subscriptions for use in sparql queries
        all_query_subs = {**self.topic_query_subs, **self.tf_query_subs}
        for name, sub in all_query_subs.items():
            kb._add_custom_function(
                NamedNode(EX + name),
                lambda s=sub: ros_msg_to_literal(s.get_latest()),
            )

        self.logger.info(
            f'SubscriberManager initialized — query-time: {list(all_query_subs.keys())}, '
            f'insertion: {list(self.insertion_subs.keys())}'
        )

    def _load_config(self, config: dict, kb: TriplestarKBInterface, env: Environment):
        for name, sub_cfg in config.get('query_time_topic_subscribers', {}).items():
            try:
                self.topic_query_subs[name] = QueryTimeTopicSubscriber(self.node, sub_cfg)
            except RuntimeError as e:
                self.logger.error(f'Failed to create topic subscriber "{name}": {e}')

        for name, sub_cfg in config.get('query_time_tf_subscribers', {}).items():
            self.tf_query_subs[name] = QueryTimeTFSubscriber(
                self.node, sub_cfg, self._buffer, self._listener
            )

        def update_fn(sparql: str) -> None:
            if kb.store is None:
                self.logger.error('KB store is not initialized, dropping insertion')
                return
            kb.store.update(sparql)

        for name, sub_cfg in config.get('insertion_subscribers', {}).items():
            try:
                self.insertion_subs[name] = InsertionSubscriber(self.node, sub_cfg, env, update_fn)
            except RuntimeError as e:
                self.logger.error(f'Failed to create insertion subscriber "{name}": {e}')
