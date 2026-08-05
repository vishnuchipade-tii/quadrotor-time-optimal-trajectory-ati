from dataclasses import dataclass
from pathlib import Path
import yaml


@dataclass
class RPGConfig:

    project_root: Path
    script: Path

    environment_type: str
    environment_name: str | None = None

    python: str = "python"

    @classmethod
    def from_yaml(cls, filename):

        with open(filename) as f:
            cfg = yaml.safe_load(f)

        root = Path(cfg["project"]["root"])

        return cls(
            project_root=root,
            script=root / cfg["script"]["entry"],
            environment_type=cfg["environment"]["type"],
            environment_name=cfg["environment"].get("name"),
            python=cfg["python"].get("executable", "python"),
        )