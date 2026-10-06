"""Train a custom YOLO target detector for the turret.

    python train.py --data datasets/turret_targets/dataset.yaml
    python train.py --data datasets/turret_targets/dataset.yaml --epochs 150 --device 0

The best weights are written to runs/detect/<name>/weights/best.pt; point
detection.model_path in host/config.yaml at a copy of that file.
See README.md in this folder for the dataset layout.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import yaml

# Mirror of host/fire_policy.py: these classes are only ever used as vetoes.
PROTECTED_CLASSES = {"person", "cat", "dog", "bird"}


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Train a YOLO model on turret targets.")
    parser.add_argument("--data", type=Path, required=True, help="dataset.yaml")
    parser.add_argument("--model", default="yolo11n.pt", help="starting weights")
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--imgsz", type=int, default=640, help="training image size")
    parser.add_argument("--batch", type=int, default=16)
    parser.add_argument("--device", default="", help='"" auto, "cpu", "0" first GPU, "mps"')
    parser.add_argument("--name", default="turret_targets", help="run name under runs/detect")
    parser.add_argument("--patience", type=int, default=30, help="early stopping patience")
    return parser.parse_args(argv)


def check_dataset(path: Path) -> list[str]:
    """Reads the class names and points out mistakes before a long training run."""
    if not path.is_file():
        raise SystemExit(f"dataset file not found: {path}")
    description = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    names = description.get("names")
    if isinstance(names, dict):
        names = [names[key] for key in sorted(names)]
    if not names:
        raise SystemExit(f"{path} defines no class names")

    protected = sorted(PROTECTED_CLASSES.intersection(names))
    if protected:
        print(
            f"note: {', '.join(protected)} will only ever veto firing in the host, "
            "never be targeted"
        )
    return list(names)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    names = check_dataset(args.data)
    print(f"classes: {', '.join(names)}")

    from ultralytics import YOLO  # heavy import, only once the arguments are valid

    model = YOLO(args.model)
    model.train(
        data=str(args.data),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device or None,
        name=args.name,
        patience=args.patience,
    )
    print(f"best weights: {model.trainer.best}")
    print("next: copy them next to the host and set detection.model_path in config.yaml")
    return 0


if __name__ == "__main__":
    sys.exit(main())
