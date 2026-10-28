"""Verify the installed launch/config assets generate valid descriptions."""
import importlib.util
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import pytest

@pytest.mark.parametrize('filename', ['lab1.launch.py', 'lab1_square.launch.py'])
def test_installed_launch(filename: str) -> None:
    """Load installed launch files, including their installed YAML dependency."""
    path = Path(get_package_share_directory('turtle_lab_bringup')) / 'launch' / filename
    spec = importlib.util.spec_from_file_location('tested_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    assert isinstance(description, LaunchDescription)
    assert description.entities
