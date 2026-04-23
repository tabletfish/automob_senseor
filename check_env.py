from __future__ import annotations

from pathlib import Path
import importlib
import os
import sys


REQUIRED_MODULES = [
    "numpy",
    "matplotlib",
    "pandas",
    "cv2",
    "open3d",
    "scipy",
    "sklearn",
    "folium",
    "ultralytics",
]


def main() -> int:
    base_dir = Path(__file__).resolve().parents[2]
    practice_dir = Path(__file__).resolve().parent
    mpl_config_dir = base_dir / ".cache" / "matplotlib"
    missing = []

    mpl_config_dir.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("MPLCONFIGDIR", str(mpl_config_dir))

    print(f"project_root: {base_dir}")
    print(f"practice_dir: {practice_dir}")
    print(f"mpl_config_dir: {os.environ['MPLCONFIGDIR']}")

    for module_name in REQUIRED_MODULES:
        try:
            importlib.import_module(module_name)
        except Exception as exc:  # pragma: no cover - environment-dependent
            missing.append((module_name, str(exc)))

    for rel_path in [
        Path("sensor_SW/data"),
        Path("sensor_SW/resources"),
        Path("sensor_SW/practice/tutlibs"),
    ]:
        target = base_dir / rel_path
        print(f"exists: {target} -> {target.exists()}")

    print(f"python: {sys.executable}")
    print(f"sys.path[0]: {sys.path[0]}")

    if missing:
        print("missing_modules:")
        for module_name, error in missing:
            print(f"  - {module_name}: {error}")
        return 1

    print("environment check passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
