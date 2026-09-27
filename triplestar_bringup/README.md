# TriplestarKB Bringup Package

This is the reference bringup package for TriplestarKB. It also bundles the
template used to generate new bringup packages.

## Generate a New Bringup Package

Rather than copying this package by hand, generate a fresh bringup package from
the bundled template using the Triplestar CLI:

```bash
ros2 triplestar bringup new
```

You will be prompted for the package name. To skip the prompt, pass it with
`--name`:

```bash
ros2 triplestar bringup new --name my_bringup
```

This renders the `bringup_template/` directory into a new package in your
workspace's `src/` folder (pass `--output-dir` to write somewhere else). The
generated package follows the folder structure described below.

## Folder Structure

TriplestarKB expects custom bringup packages to keep the generated directory structure, including `config/triplestar.yaml`.

## Config Files

All settings live in `config/triplestar.yaml`:

```yaml
knowledge_base:
  store_path: "/tmp/triplestar_kb"
  base_iri: "http://triplestar.local"
  clear_on_startup: true
  preload_files: []

insertion_subscribers:
  - topic: "/detections"
    template: "detections.sparql.tmpl"

query_time_topic_subscribers:
  - topic: "/robot_status"
    sparql_fn_name: "statusStamp"
    target_msg_field: "header.stamp"  # Optional; dotted nested paths are supported.

# TF positions need no subscriber configuration. Query them with
# qt:tfPosition(frame, referenceFrame).

query_services:
  - query_file: "get_robot_pose.sparql"
    service_name: "robot_pose"
```

## Queries

Put [SPARQL](https://www.w3.org/TR/sparql12-query/) queries in this folder.
These queries can be exposed as services through the `query_services` list in `config/triplestar.yaml`.

## Preload

Put ttl files with information you want preloaded into the knowledge base here.

**WARNING**: if you are using triple annotations (The new feature in RDF 1.2), make sure to use [explicit reifiers](https://www.w3.org/TR/rdf12-turtle/#ex-reified-triple-with-reifier). If this is not done the KB will create new blank nodes for the reifier on each startup, causing unwanted duplication of information.

## Templates

Put your [SPARQL](https://www.w3.org/TR/sparql12-query/) insertion templates in this folder.
You can use [jinja2](https://jinja.palletsprojects.com/en/stable/) syntax (so fields are surrounded by double curly braces).
Converting `ROS` message values to `RDF` values can be done using the `rdf` filter function.

For instance, the following will convert a ROS pose to a point (see [datatype conversions](../README.md#ros-to-rdf-conversions)).

```jinja2
{{ msg.pose | rdf }}
```
