"""Freeze the launch boundary that preserves the historical JACO2 stack."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.utilities import perform_substitutions
import pytest

from thesis_multi_robot import profiles


LAUNCH_DIR = Path(__file__).parents[1] / 'launch'


def _load_launch(filename, module_name):
    spec = importlib.util.spec_from_file_location(
        module_name, LAUNCH_DIR / filename
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _source_path(action, context):
    source = action.launch_description_source
    source.get_launch_description(context)
    return source.location


def test_historical_wrapper_defaults_to_jaco2():
    module = _load_launch(
        'historical_robot.launch.py', 'historical_default_launch'
    )
    description = module.generate_launch_description()
    argument = next(
        entity for entity in description.entities
        if getattr(entity, 'name', None) == 'robot'
    )
    assert perform_substitutions(
        LaunchContext(), argument.default_value
    ) == 'jaco2'


@pytest.mark.parametrize(
    ('robot', 'expected_launch'),
    [('jaco2', 'thesis_system.launch.py'), ('ur17g', 'cell.launch.py')],
)
def test_historical_wrapper_routes_jaco2_away_from_generic_cell(
        robot, expected_launch):
    module = _load_launch(
        'historical_robot.launch.py', f'historical_route_{robot}'
    )
    context = LaunchContext()
    context.launch_configurations['robot'] = robot
    action = module.select_robot_launch(context)[0]
    assert _source_path(action, context).endswith(expected_launch)


def test_internal_cell_rejects_generic_historical_jaco2(monkeypatch):
    module = _load_launch('cell.launch.py', 'generic_cell_guard_launch')

    def unexpected_build(*_args, **_kwargs):
        raise AssertionError('generic JACO2 description must not be built')

    monkeypatch.setattr(profiles, 'build_cell', unexpected_build)
    context = LaunchContext()
    context.launch_configurations.update({
        'robot': 'jaco2',
        'headless': 'true',
        'supervision': 'false',
        'historical_stack': 'true',
    })
    with pytest.raises(ValueError, match='historical_robot.launch.py'):
        module.launch_cell(context)


def test_generated_runtime_is_removed_when_profile_build_fails(
        tmp_path, monkeypatch):
    module = _load_launch('cell.launch.py', 'generic_cell_cleanup_launch')
    runtime = tmp_path / 'generated_runtime'

    def make_runtime(*_args, **_kwargs):
        runtime.mkdir()
        return str(runtime)

    def failed_build(*_args, **_kwargs):
        (runtime / 'partial-output').write_text('incomplete')
        raise RuntimeError('synthetic profile build failure')

    monkeypatch.setattr(module.tempfile, 'mkdtemp', make_runtime)
    monkeypatch.setattr(profiles, 'build_cell', failed_build)
    context = LaunchContext()
    context.launch_configurations.update({
        'robot': 'ur17g',
        'headless': 'true',
        'supervision': 'false',
        'historical_stack': 'false',
        'runtime_dir': '',
    })
    with pytest.raises(RuntimeError, match='synthetic profile build failure'):
        module.launch_cell(context)
    assert not runtime.exists()
