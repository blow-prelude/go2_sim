# Outdoor world model assets

These 14 model directories contain the external resources referenced by
`../worlds/outdoor.world`, so that the world can load without downloading models
from the Gazebo Classic model database or relying on `~/.gazebo/models`.

- Source: https://github.com/osrf/gazebo_models
- Revision: `8163eb4b5e7e21985c6591d1c0bfb56468c0093f`
- License: Creative Commons Attribution 3.0 Unported; see [LICENSE](LICENSE).
- The upstream model files are unmodified. Original author information is retained
  in each model's `model.config`.

CMake installs this directory to `share/go2w_config/models`. The package exports
`${prefix}/models` via `gazebo_ros`, which supplies the model search path to both
`gzserver` and `gzclient`. Source the workspace after building; no manual
`GAZEBO_MODEL_PATH` or `AMENT_PREFIX_PATH` edits are needed.

Only the dependencies of `outdoor.world` are bundled here. The other non-default
world, `playground.world`, requires additional models that are not included.

Run the resource regression checks with:

```bash
colcon test --packages-select go2w_config --event-handlers console_direct+
colcon test-result --verbose
```
