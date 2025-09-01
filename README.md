# TriplestarKB

TriplestarKB is a ROS2-enabled knowledge base, backed by the [oxigraph](https://github.com/oxigraph/oxigraph) SPARQL graph database.

## ROS to RDF conversions

ROS(2) defines its own set of interfaces (messages and services) for representing data like timestamps, points, integers, floats, polygons etc.
To facilitate the integration of these datatypes into the kb, which is based on RDF, these types need to be converted to suitable RDF types.

## Query-time subscribers

Some data, such as room geometries or class hirarchies, will be quite static in your KB, while other data, such as the robots own location or battery level, will change frequently.
For this requently-chaning information it is possible to add _query_time_subscribers_ to the kb node, which keep track of the messages published on a certain topic and expose them to oxigraph's underlying SPARQL evaluator to be run at query time.
As an example:
```sparql
PREFIX ex: <http://example.org/>
SELECT ?bl WHERE {
  BIND(ex:robotBatteryLevel() AS ?bl) .
  FILTER(?bl > 0.2)
}
```




- *Pellissier Tanon, T.* (n.d.). **Oxigraph**. [![DOI:10.5281/zenodo.7408022](https://zenodo.org/badge/DOI/10.5281/zenodo.7408022.svg)](https://doi.org/10.5281/zenodo.7408022)
