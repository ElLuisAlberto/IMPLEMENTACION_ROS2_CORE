#!/usr/bin/env python3
"""Verify the staged PincOpen visual assets against their source manifest."""

from __future__ import annotations

import hashlib
from pathlib import Path
import sys


EXPECTED = {
    'Bottom_Plate.stl': (2950484, '83335f57c2b5c2e6a0a9f2197908debe3492d825d864ba744772fa42c8e43b88'),
    'Cam.stl': (522284, '1fbb2277df16643444b35aa46078f8d2b4a4a991cd8bc130379f0eb337c3d6c3'),
    'Camera_basement.stl': (588384, 'eea097b2721e8c07ea930d2f37913b60b3be9ca647919a881db639ecb7310605'),
    'Camera_mobile_sup.stl': (164284, '1578ef43db974922b419ee6ca7a558be6b403ce76848ec3520d237a930555eab'),
    'Distal_Rod.stl': (187884, '5f277a3713a81e2576d08e5f6045aeb431cb3c9ac5623456be77170530622245'),
    'Distal_Shell.stl': (1482984, 'c7a5d75292e13b56f853c79f2260e5fa9376048a2ae5005234ba20e99a45f5d0'),
    'Driving_Rod.stl': (213884, '7eb6466ce94aaac561d1d18c0c1caabb3e6ef0b3906e961c61b736049a8ea240'),
    'External_Rod.stl': (116684, 'c62d7ed521d318eba93c006aa51c5abf12756c2b88d86a95f7a906d8b3f055f4'),
    'Interface_ARM100.stl': (637684, 'baa2c9de8cd8768ad15b3f827f3a992b424b2e83e77ac153ab77371444f054da'),
    'Internal_Rod.stl': (789884, '2e9a95e483899713f8121ff064df31b6e0a1f4532bf965c3112c7912534d8b68'),
    'Motor_Flange.stl': (161884, '232f90ed3cdf3dd35112e8ba15060e83aff59b975b690720ac8a32e104b75c56'),
    'Removable_Tip.stl': (311084, 'e87e1830001a2b68872850710a8aa786757e043cf2a993a07184662fb9da3194'),
    'Tip_Support.stl': (2160784, 'ed9a6f752d4c7c3ccd9b646d664b4c85c0599042028d28516cc233e158b9d7fe'),
    'Top_Plate.stl': (2707984, '953a8381c7db1216d30b81d6f83ea830949b20f24ac4ea04322b456a0bfee7c4'),
}


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> int:
    root = Path(__file__).resolve().parents[2]
    mesh_dir = root / 'src/thesis_description/meshes/pincopen/original_mm'
    failures = []
    for name, (expected_size, expected_digest) in EXPECTED.items():
        path = mesh_dir / name
        if not path.is_file():
            failures.append(f'missing: {path}')
            continue
        if path.stat().st_size != expected_size:
            failures.append(f'size mismatch: {path}')
        if digest(path) != expected_digest:
            failures.append(f'SHA-256 mismatch: {path}')
    unexpected = sorted(path.name for path in mesh_dir.glob('*.stl')
                        if path.name not in EXPECTED)
    failures.extend(f'unexpected STL: {name}' for name in unexpected)
    if failures:
        print('\n'.join(failures), file=sys.stderr)
        return 1
    print(f'PincOpen visual assets verified: {len(EXPECTED)} STL files.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
