#!/usr/bin/env bash
set -euo pipefail
segment_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
workspace_dir=$(cd "$segment_dir/../../.." && pwd)
build_dir="$workspace_dir/build/experimental_arm_segmentation"
if [[ ! -x "$build_dir/segmentation_node" || ! -x "$build_dir/gpu_mask" ]]; then
  printf '%s\n' 'Build the experimental filter first; see its README.' >&2
  exit 1
fi
export THESIS_SEGMENT_MESH="$segment_dir/mesh_points.bin"
export THESIS_SEGMENT_CAPSULES="$segment_dir/jaco_capsules_experimental.yaml"
export THESIS_SEGMENT_GPU="$build_dir/gpu_mask"
cd "$workspace_dir"
exec "$build_dir/segmentation_node" --ros-args \
  -r /codex/segmentacion_fused/brazo:=/codex/segmentacion/brazo_candidato \
  -r /codex/segmentacion_fused/incierto:=/codex/segmentacion/margen_incierto \
  -r /codex/segmentacion_fused/resto:=/codex/segmentacion/resto_candidato "$@"
