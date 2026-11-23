# Perception scripts

The scripts are grouped by their role:

```text
scripts/
├── point_flow_preview.py
├── point_flow_raft_preview_experimental.py
├── distance_comparison_views.py
└── alternativas/
    └── point_flow_raft_preview.py
```

## Current runtime

- `point_flow_preview.py` is the shared RGB-D base used by both RAFT
  previews. It is not a standalone replacement for the predictors.
- `point_flow_raft_preview_experimental.py` is the current CUDA RAFT
  predictor. It is the only RAFT predictor used by the current session.
- `distance_comparison_views.py` publishes RGB-D XYZRGB clouds filtered by
  distance and after bilateral smoothing.

## Comparison implementation

- `alternativas/point_flow_raft_preview.py` is the previous RAFT predictor.
  It remains available for reproducible comparisons and is not started by
  the current launch configuration.

The alternative must publish to its own visualization topics when it is
compared with the current predictor. Do not run both predictors against the
same output topics at the same time.

The installed executable names remain unchanged so existing commands and
recorded experiments continue to work.
