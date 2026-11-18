"""Load and validate paths used by the ROS 2 launch files."""

from pathlib import Path
from typing import Tuple

import yaml
from ament_index_python.packages import get_package_share_directory


def get_launch_paths(world_variant: str = "default") -> Tuple[Path, Path, Path]:
    """Return simulator, PX4, and world paths from the installed YAML config."""
    config_path = (
        Path(get_package_share_directory("pose_estimator_pkg"))
        / "launch_config.yaml"
    )
    try:
        with config_path.open(encoding="utf-8") as config_file:
            config = yaml.safe_load(config_file) or {}
    except OSError as exc:
        raise RuntimeError(
            f"Could not read {config_path}. Generate the config with "
            "python3 scripts/generate_launch_config.py and rebuild the package."
        ) from exc
    except yaml.YAMLError as exc:
        raise RuntimeError(f"Invalid YAML in {config_path}: {exc}") from exc

    if not isinstance(config, dict):
        raise RuntimeError(f"Expected a YAML mapping in {config_path}.")
    environment = config.get("sim_environment")
    if not isinstance(environment, dict):
        raise RuntimeError(
            f"Missing 'sim_environment' section in {config_path}. "
            "Regenerate launch_config.yaml."
        )

    sim_repo = _configured_path(environment, "sim_repo_path", config_path)
    px4_repo = _configured_path(environment, "px4_repo_path", config_path)
    world_key = "world_name" if world_variant == "default" else "simple_world_name"
    world_name = environment.get(world_key)
    if not isinstance(world_name, str) or not world_name.strip():
        raise RuntimeError(
            f"Missing '{world_key}' in {config_path}. "
            "Regenerate launch_config.yaml."
        )

    world_path = Path(world_name).expanduser()
    if not world_path.is_absolute():
        world_path = sim_repo / world_path
    world_path = world_path.resolve()

    repositories = (
        ("simulator repository", sim_repo),
        ("PX4 repository", px4_repo),
    )
    for label, path in repositories:
        if not path.is_dir():
            raise RuntimeError(f"The configured {label} does not exist: {path}")
    if not world_path.is_file():
        raise RuntimeError(f"The configured world file does not exist: {world_path}")

    return sim_repo, px4_repo, world_path


def _configured_path(environment: dict, key: str, config_path: Path) -> Path:
    """Read a non-empty path setting and resolve it to an absolute path."""
    value = environment.get(key)
    if not isinstance(value, str) or not value.strip():
        raise RuntimeError(
            f"Missing '{key}' in {config_path}. "
            "Regenerate launch_config.yaml."
        )
    return Path(value).expanduser().resolve()