from pathlib import Path

from pydantic import BaseModel
from pydantic import Field
from pydantic import root_validator
from pydantic import validator


class KBConfig(BaseModel):
    store_path: Path
    preload_files: list[str] = Field(default_factory=list)
    base_iri: str
    clear_on_startup: bool = True


class InsertionSubscriberConfig(BaseModel):
    topic: str
    template: str


class QueryTimeTopicSubscriberConfig(BaseModel):
    topic: str
    sparql_fn_name: str
    target_msg_field: str | None = None


class QueryTimeTFSubscriberConfig(BaseModel):
    from_frame: str
    to_frame: str
    sparql_fn_name: str


class QueryServiceConfig(BaseModel):
    query_file: str
    service_name: str


class TriplestarConfig(BaseModel):
    knowledge_base: KBConfig
    insertion_subscribers: list[InsertionSubscriberConfig] = Field(default_factory=list)
    query_time_topic_subscribers: list[QueryTimeTopicSubscriberConfig] = Field(
        default_factory=list
    )
    query_time_tf_subscribers: list[QueryTimeTFSubscriberConfig] = Field(default_factory=list)
    query_services: list[QueryServiceConfig] = Field(default_factory=list)

    @validator(
        'insertion_subscribers',
        'query_time_topic_subscribers',
        'query_time_tf_subscribers',
        'query_services',
        pre=True,
    )
    @classmethod
    def _none_to_empty_list(cls, value):
        """Treat an empty YAML key (parsed as None) the same as an omitted one."""
        return value if value is not None else []

    @root_validator
    def _unique_names(cls, values):  # noqa: N805
        query_time_names = [
            subscriber.sparql_fn_name
            for subscriber in values.get('query_time_topic_subscribers', [])
        ] + [
            subscriber.sparql_fn_name for subscriber in values.get('query_time_tf_subscribers', [])
        ]
        if len(query_time_names) != len(set(query_time_names)):
            raise ValueError('Query-time SPARQL function names must be unique')

        service_names = [service.service_name for service in values.get('query_services', [])]
        if len(service_names) != len(set(service_names)):
            raise ValueError('Query service names must be unique')

        return values
