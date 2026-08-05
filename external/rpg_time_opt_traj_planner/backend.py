from pathlib import Path
import subprocess
import yaml

from external.rpg_time_opt_traj_planner.exporter import save_rpg_track
from external.rpg_time_opt_traj_planner.importer import load_rpg_result
from external.rpg_time_opt_traj_planner.config import RPGConfig


class RPGTimeOptimalTrajectoryPlannerBackend:

    def __init__(self, config: RPGConfig):
        self.config = config

    @classmethod
    def from_yaml(cls, filename):
        return cls(RPGConfig.from_yaml(filename))

    def _build_command(
        self,
        quad_yaml: Path,
        track_yaml: Path,
        output_dir: Path,
    ):

        if self.config.environment_type == "conda":
            return [
                "conda",
                "run",
                "--name",
                self.config.environment_name,
                self.config.python,
                str(self.config.script),
                "--quad",
                str(quad_yaml),
                "--track",
                str(track_yaml),
                "--output",
                str(output_dir),
            ]

        if self.config.environment_type == "venv":
            return [
                self.config.python,
                str(self.config.script),
                "--quad",
                str(quad_yaml),
                "--track",
                str(track_yaml),
                "--output",
                str(output_dir),
            ]

        raise ValueError(
            f"Unknown environment '{self.config.environment_type}'"
        )

    def solve(
        self,
        *,
        quad_config,
        experiment_config,
        output_root: Path,
    ):
        """
        Execute the RPG planner and return the optimized trajectory.
        """

        workdir = output_root / experiment_config["name"]
        workdir.mkdir(parents=True, exist_ok=True)

        quad_yaml = workdir / "quad.yaml"
        track_yaml = workdir / "track.yaml"
        result_dir = workdir / "results"

        with open(quad_yaml, "w") as f:
            yaml.safe_dump(
                quad_config,
                f,
                sort_keys=False,
            )

        save_rpg_track(
            experiment_config,
            track_yaml,
        )

        command = self._build_command(
            quad_yaml,
            track_yaml,
            result_dir,
        )

        subprocess.run(
            command,
            check=True,
        )

        return load_rpg_result(
            result_dir / "result.csv"
        )