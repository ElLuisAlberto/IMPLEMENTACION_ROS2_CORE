"""Doosan P3020 nominal geometry for the thesis Gazebo backend.

The manufacturer's static URDF includes collision meshes, unlike its Gazebo
Xacro at the pinned revision.  Loading the static file also avoids starting the
Doosan controller or its Docker emulator.  Joint origins, masses, inertias,
visuals and all 22 collision geometries remain those of the upstream model.
"""

from __future__ import annotations

import math
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory


JOINT_NAMES = ('joint_1', 'joint_2', 'joint_3', 'joint_5', 'joint_6')
BASE_LINK = 'base_link'
TIP_LINK = 'link_6'
INITIAL_POSITIONS = {name: 0.0 for name in JOINT_NAMES}
SOURCE_PACKAGES = ('dsr_description2',)
DEFAULT_VISUAL_VARIANT = 'blue'
VISUAL_VARIANTS = ('steel_preview', 'blue', 'white')
# Brighter steel blue for legibility in RViz; an approximation inspired by
# the reference image, not an exact manufacturer's finish. The original
# manufacturer blue remains available as the explicit 'blue' variant.
PREVIEW_BLUE_RGBA = '0.20 0.32 0.47 1'


def _official_visual_palette(
    share: Path, visual_variant: str,
) -> dict[tuple[str, str], str]:
    """Return the official Doosan colour assigned to every visual mesh.

    The colour-specific upstream URDFs omit collision geometry, so they must
    not replace ``p3020.urdf``.  They are used only as an authoritative palette
    for the collision-complete model.
    """
    if visual_variant not in VISUAL_VARIANTS:
        raise ValueError(
            f'Unsupported P3020 visual variant {visual_variant!r}; '
            f'expected one of {VISUAL_VARIANTS}'
        )
    source_variant = (
        'blue' if visual_variant == 'steel_preview' else visual_variant
    )
    palette_root = ET.parse(
        share / 'urdf' / f'p3020.{source_variant}.urdf'
    ).getroot()
    named_colors = {
        material.get('name'): material.find('color').get('rgba')
        for material in palette_root.findall('material')
        if material.get('name') and material.find('color') is not None
    }
    result: dict[tuple[str, str], str] = {}
    for link in palette_root.findall('link'):
        for visual in link.findall('visual'):
            mesh = visual.find('geometry/mesh')
            material = visual.find('material')
            if mesh is None or material is None:
                raise ValueError('P3020 palette contains an incomplete visual')
            rgba = (
                material.find('color').get('rgba')
                if material.find('color') is not None
                else named_colors.get(material.get('name'))
            )
            if rgba is None:
                raise ValueError(
                    'P3020 palette material has no colour: '
                    f'{material.get("name")!r}'
                )
            if (visual_variant == 'steel_preview' and
                    tuple(map(float, rgba.split())) == (0., 0., 0.15, 1.)):
                rgba = PREVIEW_BLUE_RGBA
            key = (link.get('name'), mesh.get('filename'))
            if key in result:
                raise ValueError(f'Duplicate P3020 palette visual: {key!r}')
            result[key] = rgba
    return result


def _apply_official_visual_palette(
    root: ET.Element, share: Path, visual_variant: str,
) -> None:
    """Apply explicit per-visual colours without changing physical content."""
    palette = _official_visual_palette(share, visual_variant)
    seen: set[tuple[str, str]] = set()
    visual_index = 0
    for link in root.findall('link'):
        for visual in link.findall('visual'):
            mesh = visual.find('geometry/mesh')
            if mesh is None:
                raise ValueError('P3020 model contains a non-mesh visual')
            key = (link.get('name'), mesh.get('filename'))
            rgba = palette.get(key)
            if rgba is None:
                raise ValueError(
                    'P3020 visual is absent from official palette: '
                    f'{key!r}'
                )
            material = visual.find('material')
            if material is None:
                material = ET.SubElement(visual, 'material')
            for child in list(material):
                material.remove(child)
            # Unique inline names prevent URDF-to-SDF from resolving only the
            # first repeated named material and rendering later visuals black.
            material.set('name', f'p3020_{visual_variant}_{visual_index:02d}')
            ET.SubElement(material, 'color', {'rgba': rgba})
            seen.add(key)
            visual_index += 1
    if seen != set(palette):
        missing = sorted(set(palette) - seen)
        raise ValueError(f'P3020 palette has unmatched visuals: {missing!r}')


def load_model(visual_variant: str = DEFAULT_VISUAL_VARIANT) -> ET.Element:
    """Return the manufacturer's five-axis URDF with collision geometry.

    This is a description adapter, not a Doosan hardware interface.  The caller
    supplies the world attachment, unique names and simulation-only control.
    Reject missing meshes or incompatible upstream revisions at load time.
    """
    share = Path(get_package_share_directory('dsr_description2'))
    root = ET.parse(share / 'urdf' / 'p3020.urdf').getroot()
    if root.tag != 'robot':
        raise ValueError('P3020 source is not a URDF robot')

    active = tuple(
        joint.get('name') for joint in root.findall('joint')
        if joint.get('type') in ('revolute', 'continuous', 'prismatic')
    )
    if active != JOINT_NAMES:
        raise ValueError(f'Unexpected P3020 active joints: {active!r}')
    fixed_fourth = root.find("joint[@name='joint_4']")
    if fixed_fourth is None or fixed_fourth.get('type') != 'fixed':
        raise ValueError('P3020 joint_4 must remain fixed (five active axes)')
    if len(root.findall('link/collision')) < 22:
        raise ValueError('P3020 collision geometry is missing or incomplete')

    _apply_official_visual_palette(root, share, visual_variant)

    for mesh in root.findall('.//mesh'):
        filename = mesh.get('filename', '')
        uri_prefix = 'package://dsr_description2/'
        if not filename.startswith(uri_prefix):
            raise ValueError(f'Unexpected P3020 mesh URI: {filename!r}')
        mesh_path = share / filename[len(uri_prefix):]
        if not mesh_path.is_file():
            raise FileNotFoundError(f'P3020 mesh not found: {mesh_path}')

    for joint_name in JOINT_NAMES:
        limit = root.find(f"joint[@name='{joint_name}']/limit")
        if limit is None:
            raise ValueError(f'P3020 joint {joint_name} has no limits')
        lower, upper, velocity, effort = (
            float(limit.get(key, 'nan'))
            for key in ('lower', 'upper', 'velocity', 'effort')
        )
        if not all(math.isfinite(value) for value in
                   (lower, upper, velocity, effort)):
            raise ValueError(f'P3020 joint {joint_name} has non-finite limits')
        if not lower <= INITIAL_POSITIONS[joint_name] <= upper:
            raise ValueError(f'P3020 initial pose violates {joint_name} limits')
        if lower >= upper or velocity <= 0.0 or effort <= 0.0:
            raise ValueError(f'P3020 joint {joint_name} has invalid limits')

    # The pinned file has no control plugins.  Keep that invariant if upstream
    # later adds defaults, because this adapter is strictly simulation-only.
    for tag in ('gazebo', 'ros2_control', 'transmission'):
        for element in root.findall(tag):
            root.remove(element)
    return root
