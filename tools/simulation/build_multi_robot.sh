#!/usr/bin/env bash
# Run from any directory; preserve both the historical JACO2 simulation and
# the new multi-robot cell in the same overlay.
set -eo pipefail
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$task_root"
set +u
source /opt/ros/humble/setup.bash
set -u
/usr/bin/python3 -c 'import numpy, scipy, yaml, trimesh, collada; assert trimesh.__version__ == "4.4.9", "Use trimesh==4.4.9 with NumPy 1.21 on Humble"'
colcon build --symlink-install \
  --base-paths \
    src/thesis_description src/thesis_interfaces src/thesis_core src/thesis_ui \
    src/thesis_simulation src/thesis_multi_robot src/sistema_gripper_instrumentado_description \
    third_party/robots/universal_robots/ros2_description \
    third_party/robots/elite/ros2_description \
    third_party/robots/doosan/doosan_robot2/dsr_description2 \
  --packages-select thesis_description thesis_interfaces thesis_core thesis_ui \
    thesis_simulation ur_description \
    dsr_description2 elite_robots_description sistema_gripper_instrumentado_description thesis_multi_robot
