#!/usr/bin/env python3
"""Generate launch_config.yaml with local simulator and PX4 paths."""

import argparse
import json
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _ask_path(label: str, default: str) -> Path:
    """Prompt for a path and ensure it points to an existing directory."""
    while True:
        answer = input(f"{label} [{default}]: ").strip() or default
        path = Path(answer).expanduser().resolve()
        if path.is_dir():
            return path
        print(f"No existe el directorio: {path}")


def _ask_world(label: str, default: str, sim_repo: Path) -> str:
    """Prompt for an SDF filename and ensure it exists in the simulator repo."""
    while True:
        answer = input(f"{label} [{default}]: ").strip() or default
        world_path = Path(answer).expanduser()
        if not world_path.is_absolute():
            world_path = sim_repo / world_path
        if world_path.is_file():
            return answer
        print(f"No existe el archivo de mundo: {world_path}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Genera launch_config.yaml para los launch de ROS 2."
    )
    parser.add_argument("--sim-repo", help="Ruta del repositorio de simulación")
    parser.add_argument("--px4-repo", help="Ruta del repositorio PX4-Autopilot")
    parser.add_argument("--world", help="Mundo SDF usado por los launch generales")
    parser.add_argument(
        "--simple-world", help="Mundo SDF usado por simple_launch.launch.py"
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=PACKAGE_ROOT / "launch_config.yaml",
        help="Archivo YAML de salida (por defecto: launch_config.yaml del paquete)",
    )
    args = parser.parse_args()

    sim_repo = (
        Path(args.sim_repo).expanduser().resolve()
        if args.sim_repo
        else _ask_path("Repositorio de simulación", "~/cats_sim")
    )
    px4_repo = (
        Path(args.px4_repo).expanduser().resolve()
        if args.px4_repo
        else _ask_path("Repositorio PX4-Autopilot", "~/PX4-Autopilot")
    )
    for label, path in (("simulación", sim_repo), ("PX4", px4_repo)):
        if not path.is_dir():
            parser.error(f"No existe el directorio de {label}: {path}")

    world_name = args.world or _ask_world("Mundo SDF", "cats_world.sdf", sim_repo)
    simple_world_name = args.simple_world or _ask_world(
        "Mundo SDF simple", "cats_no_collision.sdf", sim_repo
    )
    worlds = (("--world", world_name), ("--simple-world", simple_world_name))
    for label, name in worlds:
        world_path = Path(name).expanduser()
        if not world_path.is_absolute():
            world_path = sim_repo / world_path
        if not world_path.is_file():
            parser.error(f"El archivo indicado por {label} no existe: {world_path}")

    output = args.output.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        "sim_environment:\n"
        f"  sim_repo_path: {json.dumps(str(sim_repo))}\n"
        f"  px4_repo_path: {json.dumps(str(px4_repo))}\n"
        f"  world_name: {json.dumps(world_name)}\n"
        f"  simple_world_name: {json.dumps(simple_world_name)}\n",
        encoding="utf-8",
    )
    print(f"Configuración guardada en: {output}")
    print("Ahora reconstruye el paquete para instalar el YAML actualizado.")


if __name__ == "__main__":
    main()