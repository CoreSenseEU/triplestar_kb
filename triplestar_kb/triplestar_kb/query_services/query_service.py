from pathlib import Path
from typing import Callable

from rclpy.lifecycle import LifecycleNode
from rclpy.node import Node
from triplestar_kb_msgs.srv import Query

"""
The query service sets up a ROS service backed by a .sparql file.
On each call it reads the file and delegates execution to query_fn.
"""


class QueryService:
    def __init__(
        self,
        node: Node | LifecycleNode,
        name: str,
        query_file: Path,
        query_fn: Callable[[str], str],
    ):
        self.name = name

        if not query_file.exists():
            raise FileNotFoundError(f'Query file not found: {query_file}')

        self.query_file = query_file
        self._query_fn = query_fn

        self._service = node.create_service(
            srv_name=name,
            srv_type=Query,
            callback=self._callback,
        )

    def _callback(
        self,
        request: Query.Request,
        response: Query.Response,
    ) -> Query.Response:
        with open(self.query_file, 'r') as f:
            query = f.read()
        response.result = self._query_fn(query)
        return response
