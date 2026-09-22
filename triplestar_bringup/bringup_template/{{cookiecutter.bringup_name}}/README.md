# {{cookiecutter.bringup_name}} — TriplestarKB Bringup Package

Configure this package in `config/triplestar.yaml`. The file contains the top-level
`knowledge_base`, `insertion_subscribers`, `query_time_topic_subscribers`,
`query_time_tf_subscribers`, and `query_services` sections. Subscriber and service
sections are lists. Query-time topic entries may use `target_msg_field` to select a
field, including a dotted nested path
such as `header.stamp`.

Place SPARQL extension functions in ``functions/`` using the ``@kb_function`` decorator.

```python
from triplestar_core.functions import kb_function

@kb_function("hello")
def hello(name) -> str:
    return f"Hello, {name.value}!"
```

Called in SPARQL as ``fn:hello("world")``.
