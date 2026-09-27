---
icon: lucide/package
---

# Bringup package structure

A bringup package is the per-scenario configuration layer for TriplestarKB. It contains everything needed to tailor the KB to your robot, environment, and use case.

## Directory layout

```
my_bringup/
├── config/
│   └── triplestar.yaml          # KB, subscriber, and query-service settings
├── preload/                     # Turtle (.ttl) files loaded at startup
├── queries/                     # SPARQL query files for query services
├── templates/                   # Jinja2 templates for insertion subscribers
├── functions/                   # Python SPARQL extension functions
├── launch/
│   └── my_bringup_triplestar.launch.xml
├── CMakeLists.txt
└── package.xml
```

## How it works

The shared KB launch file (`triplestar_kb.launch.py`) takes the custom package name as its `bringup-package` argument. At configure time, the `TriplestarKBNode` lifecycle node:

1. Resolves the bringup package's share directory via `ament_index_python.get_package_share_directory()`
2. Loads the unified `config/triplestar.yaml`
3. Preloads the configured `.ttl` files from `preload/`
4. Starts insertion subscribers, query-time subscribers, and query services
5. Discovers and registers custom SPARQL functions from `functions/`

## Launch file

The generated launch file wraps the shared `triplestar_kb.launch.py` and sets the `bringup-package` argument:

```xml
<launch>
  <include file="$(find-pkg-share triplestar_bringup)/launch/triplestar_kb.launch.py">
    <arg name="bringup-package" value="my_bringup" />
  </include>
</launch>
```

## Creating a bringup package

Use the Triplestar CLI, which renders the bundled Cookiecutter template into
the active workspace's `src/` directory:

```bash
ros2 triplestar bringup new --name my_bringup
```

This scaffolds the full directory structure with example files you can customize.
