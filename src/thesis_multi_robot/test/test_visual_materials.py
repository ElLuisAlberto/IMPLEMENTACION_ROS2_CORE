"""Regression tests for manufacturer visual materials."""

from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
from ament_index_python.packages import get_package_share_directory

from thesis_multi_robot.models import p3020, ur17g


def _rgba_counts(root):
    counts = {}
    for color in root.findall('link/visual/material/color'):
        rgba = color.get('rgba')
        counts[rgba] = counts.get(rgba, 0) + 1
    return counts


def test_p3020_steel_preview_uses_readable_steel_and_white():
    root = p3020.load_model('steel_preview')
    visuals = root.findall('link/visual')
    materials = root.findall('link/visual/material')
    assert len(visuals) == len(materials) == 22
    assert len({material.get('name') for material in materials}) == 22
    assert all(material.find('color') is not None for material in materials)
    assert _rgba_counts(root) == {
        '0.20 0.32 0.47 1': 20,
        '0.941 0.941 0.941 1.0': 2,
    }
    assert len(root.findall('link/collision')) == 22


def test_p3020_default_remains_official_blue():
    assert _rgba_counts(p3020.load_model()) == {
        '0 0 0.15 1': 20,
        '0.941 0.941 0.941 1.0': 2,
    }


def test_p3020_white_variant_preserves_collision_complete_model():
    root = p3020.load_model('white')
    assert _rgba_counts(root) == {'0.941 0.941 0.941 1.0': 22}
    assert len(root.findall('link/visual')) == 22
    assert len(root.findall('link/collision')) == 22


def test_p3020_palette_changes_only_visual_materials():
    share = Path(get_package_share_directory('dsr_description2'))
    original = ET.parse(share / 'urdf' / 'p3020.urdf').getroot()
    adapted = p3020.load_model()
    for root in (original, adapted):
        for visual in root.findall('link/visual'):
            material = visual.find('material')
            if material is not None:
                visual.remove(material)
        for material in root.findall('material'):
            root.remove(material)
    assert ET.tostring(adapted) == ET.tostring(original)


def test_p3020_rejects_unknown_visual_variant():
    with pytest.raises(ValueError, match='Unsupported P3020 visual variant'):
        p3020.load_model('orange')


def test_ur17g_keeps_embedded_dae_materials_without_monochrome_override():
    root = ur17g.load_model()
    visuals = root.findall('link/visual')
    assert len(visuals) == 8
    assert not root.findall('link/visual/material/color')
    assert all(
        visual.find('geometry/mesh').get('filename').endswith(('.dae', '.stl'))
        for visual in visuals
    )
