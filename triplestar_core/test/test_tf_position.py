import json
import logging

from geometry_msgs.msg import TransformStamped
from rclpy.clock import Clock
from rclpy.clock import ClockType
from rclpy.time import Time
from rdflib.namespace import GEO
from shapely import wkt
import tf2_ros
from triplestar_core.knowledge_base import KnowledgeBase
from triplestar_core.subscriptions.query_time_subscriber import TransformPositionLookup
from triplestar_core.subscriptions.subscriber_manager import make_tf_position_query_fn


def _transform(
    parent: str,
    child: str,
    translation: tuple[float, float, float],
    stamp: Time,
) -> TransformStamped:
    transform = TransformStamped()
    transform.header.frame_id = parent
    transform.header.stamp = stamp.to_msg()
    transform.child_frame_id = child
    transform.transform.translation.x = translation[0]
    transform.transform.translation.y = translation[1]
    transform.transform.translation.z = translation[2]
    transform.transform.rotation.w = 1.0
    return transform


def _lookup(buffer: tf2_ros.Buffer, clock: Clock) -> TransformPositionLookup:
    return TransformPositionLookup(
        buffer=buffer,
        clock=clock,
        logger=logging.getLogger('test.tf_position'),
    )


def test_tf_position_returns_point_z_wkt_for_known_frame():
    clock = Clock(clock_type=ClockType.SYSTEM_TIME)
    buffer = tf2_ros.Buffer()
    buffer.set_transform(
        _transform('map', 'base_link', (1.0, 2.0, 3.0), clock.now()),
        'test',
    )
    kb = KnowledgeBase(store_path=None, base_iri='http://example.org')
    kb.add_query_time_function(
        'tfPosition',
        make_tf_position_query_fn(_lookup(buffer, clock)),
    )

    query_result = kb.query(
        'SELECT ?position WHERE { '
        'BIND(qt:tfPosition("base_link", "map") AS ?position) '
        '}'
    )

    assert isinstance(query_result, str)
    result = json.loads(query_result)['results']['bindings'][0]['position']
    assert result['datatype'] == str(GEO.wktLiteral)
    point = wkt.loads(result['value'])
    assert point.has_z
    assert tuple(point.coords[0]) == (1.0, 2.0, 3.0)


def test_tf_position_resolves_chained_frames():
    clock = Clock(clock_type=ClockType.SYSTEM_TIME)
    buffer = tf2_ros.Buffer()
    stamp = clock.now()
    buffer.set_transform(_transform('map', 'odom', (1.0, 2.0, 3.0), stamp), 'test')
    buffer.set_transform(_transform('odom', 'base_link', (4.0, 5.0, 6.0), stamp), 'test')

    position = _lookup(buffer, clock).get_position('base_link', 'map')

    assert position is not None
    assert (position.x, position.y, position.z) == (5.0, 7.0, 9.0)


def test_tf_position_leaves_result_unbound_for_unknown_frame():
    clock = Clock(clock_type=ClockType.SYSTEM_TIME)
    kb = KnowledgeBase(store_path=None, base_iri='http://example.org')
    kb.add_query_time_function(
        'tfPosition',
        make_tf_position_query_fn(_lookup(tf2_ros.Buffer(), clock)),
    )

    query_result = kb.query(
        'SELECT ?position WHERE { '
        'BIND(qt:tfPosition("missing", "map") AS ?position) '
        '}'
    )

    assert isinstance(query_result, str)
    assert json.loads(query_result)['results']['bindings'] == [{}]


def test_tf_position_returns_none_for_stale_transform():
    clock = Clock(clock_type=ClockType.SYSTEM_TIME)
    buffer = tf2_ros.Buffer()
    stale_stamp = Time(
        nanoseconds=clock.now().nanoseconds - 3_000_000_000,
        clock_type=ClockType.SYSTEM_TIME,
    )
    buffer.set_transform(
        _transform('map', 'base_link', (1.0, 2.0, 3.0), stale_stamp),
        'test',
    )

    assert _lookup(buffer, clock).get_position('base_link', 'map') is None
