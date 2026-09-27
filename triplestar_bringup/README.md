# TriplestarKB Bringup Package

`triplestar_bringup` provides the shared launch file that starts the TriplestarKB
lifecycle node, configures and activates it, and optionally starts the geometry
visualizer. Generated scenario packages include this launch file and select their
own configuration with the `bringup-package` argument.

It also bundles the template used by the Triplestar CLI to create those scenario
packages:

```bash
ros2 triplestar bringup new
ros2 triplestar bringup new --name my_bringup
```

By default, the generated package is written to the active workspace's `src/`
directory. Pass `--output-dir` to choose another location.

See the [bringup configuration documentation](../docs/docs/bringup_package/config-files.md)
for the generated package layout and configuration options.
