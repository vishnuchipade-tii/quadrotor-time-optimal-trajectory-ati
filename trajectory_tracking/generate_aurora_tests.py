import json
import argparse
import shutil

from pathlib import Path
from typing import Dict, Any
from dataclasses import dataclass


# ============================================================
# Data types
# ============================================================

@dataclass(frozen=True)
class CsvFile:
    full_path: Path
    relative_path: Path


# ============================================================
# Constants
# ============================================================

WS_TOKEN = "{{WS_ROOT}}"


# ============================================================
# Templates
# ============================================================

TRAJ_TEMPLATE = {
    "traj_type": "ros_topic",
    "start_time": 0.0,
    "sampling_time": 0.01,
    "end_time": 50.0,
    "ros_node_name": "trajectory_publisher",
    "ros_package_name": "trajectory_publisher",
    "ros_node_launch_sh_file": (
        f"{WS_TOKEN}/src/trajectory_publisher/"
        "launch/trajectory_publisher_launch.sh"
    ),
    "ros_node_args": [],
    "ros_topic_name": "/trajectory/waypoint",
    "controller_id": ""
}


TEST_TEMPLATE = {
    "test_name": "PLACEHOLDER",
    "uav_type": ["quadcopter", "multicopter"],
    "model": ["x500"],
    "test_filter": ["offboard"],
    "ground_truth_recorder": {
        "enabled": True,
        "input_data_file": ""
    },
    "actions": [
        {"action": "set_offboard_mode", "time_out_sec": 10},
        {"action": "arm", "time_out_sec": 10},
        {"action": "takeoff", "takeoff_altitude": -2.0, "time_out_sec": 10},
        {
            "action": "track_traj",
            "traj_type": "ros2_topic",
            "input_data_file": "",
            "time_out_sec": 80
        },
        {"action": "land", "time_out_sec": 20}
    ]
}


CONFIG_TEMPLATE = {
    "mode": "sitl",
    "simulator": "gazebo-harmonic",
    "world": "default",
    "mavlink_connection": "udp://:14540",
    "px4_dir": "{{PX4_PATH}}",
    "px4_build_dir": "build/px4_sitl_default/",
    "tests_dir": "",
    "tests_build_dir": f"{WS_TOKEN}/build",
    "tests": [
        {
            "model": "x500",
            "test_filter": "[multicopter],[offboard],[offboard_attitude]",
            "timeout_min": 10
        }
    ]
}


GROUND_TRUTH_RECORDER_TEMPLATE = {
    "gz_topic": "/world/default/pose/info",
    "model_name": "x500",
    "output_csv": "gz_ground_truth.csv",
    "ros_node_name": "gz_ground_truth_recorder",
    "ros_node_launch_sh_file": (
        f"{WS_TOKEN}/src/gz_ground_truth_recorder/"
        "launch/gz_ground_truth_recorder_launch.sh"
    ),
    "ros_node_args": [],
}


# ============================================================
# Test Generator Class
# ============================================================

class TestCaseGenerator:

    def __init__(
        self,
        input_dir: Path,
        output_root: Path,
        container_root: str | None = None,
    ):
        self.input_dir = input_dir.resolve()
        self.output_root = output_root.resolve()
        self.container_root = container_root

        self.traj_output_dir = (
            self.output_root / "config" / "trajectory"
        )

        self.test_output_dir = (
            self.output_root / "test"
        )

        self.config_output_dir = (
            self.output_root / "config"
        )

        self.gt_config_output_dir = (
            self.output_root / "config" / "io"
        )

        self.gt_result_output_dir = (
            self.output_root / "results"
        )


    # --------------------------------------------------------

    @staticmethod
    def write_json(path: Path, data: Dict[str, Any]):
        path.parent.mkdir(parents=True, exist_ok=True)

        with open(path, "w") as f:
            json.dump(data, f, indent=2)


    # --------------------------------------------------------

    @staticmethod
    def find_csv_files(root: Path):
        return [
            CsvFile(
                full_path=p,
                relative_path=p.relative_to(root)
            )
            for p in root.rglob("trajectory.csv")
        ]


    # --------------------------------------------------------

    def runtime_path(self, path: Path) -> str:
        """
        Convert host generated path into container path.

        If no container root is supplied,
        return normal filesystem path.
        """

        if self.container_root is None:
            return path.as_posix()

        relative = path.relative_to(self.output_root)

        return (
            Path(self.container_root) / relative
        ).as_posix()


    # --------------------------------------------------------

    @staticmethod
    def make_test_name(csv: CsvFile):

        name = "_".join(csv.relative_path.parts[:-1])

        return (
            f"Offboard_traj_track_"
            f"{name.replace('/', '_')}"
        )


    # --------------------------------------------------------

    def generate_traj_json(
        self,
        csv: CsvFile,
    ) -> Path:

        out_csv = (
            self.traj_output_dir /
            csv.relative_path
        )

        out_json = out_csv.with_suffix(".json")

        out_csv.parent.mkdir(
            parents=True,
            exist_ok=True
        )

        shutil.copy2(
            csv.full_path,
            out_csv
        )

        traj = json.loads(
            json.dumps(TRAJ_TEMPLATE)
        )

        traj["ros_node_args"] = (
            self.runtime_path(out_csv)
        )

        self.write_json(
            out_json,
            traj
        )

        print(f"[TRAJ] {out_json}")

        return out_json


    # --------------------------------------------------------

    def generate_ground_truth_recorder_config(
        self,
        csv: CsvFile,
        model_name="x500",
        output_csv="gz_ground_truth.csv",
        gz_topic="/world/default/pose/info",
    ) -> Path:

        config = json.loads(
            json.dumps(GROUND_TRUTH_RECORDER_TEMPLATE)
        )

        config["model_name"] = model_name
        config["output_csv"] = output_csv
        config["gz_topic"] = gz_topic


        result_dir = (
            self.gt_result_output_dir /
            csv.relative_path.parent
        )

        result_dir.mkdir(
            parents=True,
            exist_ok=True
        )

        config["output_dir"] = (
            self.runtime_path(result_dir)
        )


        config_path = (
            self.gt_config_output_dir /
            csv.relative_path.parent /
            "gz_ground_truth_recorder.json"
        )

        self.write_json(
            config_path,
            config
        )

        print(f"[GT CONFIG] {config_path}")

        return config_path


    # --------------------------------------------------------

    def generate_test_json(
        self,
        csv: CsvFile,
        traj_json: Path,
        gt_config: Path,
    ):

        test = json.loads(
            json.dumps(TEST_TEMPLATE)
        )

        test["test_name"] = (
            self.make_test_name(csv)
        )

        test["actions"][3]["input_data_file"] = (
            self.runtime_path(traj_json)
        )

        test["ground_truth_recorder"]["input_data_file"] = (
            self.runtime_path(gt_config)
        )


        out_file = (
            self.test_output_dir /
            csv.relative_path.parent /
            "test.json"
        )

        self.write_json(
            out_file,
            test
        )

        print(f"[TEST] {out_file}")


    # --------------------------------------------------------

    def generate_config_json(
        self,
        vehicle="x500",
    ):

        config = json.loads(
            json.dumps(CONFIG_TEMPLATE)
        )

        config["tests_dir"] = (
            self.runtime_path(
                self.test_output_dir
            )
        )

        config_path = (
            self.config_output_dir /
            f"config_{vehicle}.json"
        )

        self.write_json(
            config_path,
            config
        )

        print(f"[CONFIG] {config_path}")


    # --------------------------------------------------------

    def generate(self):

        csv_files = self.find_csv_files(
            self.input_dir
        )

        if not csv_files:
            print(
                f"No CSV files found in {self.input_dir}"
            )
            return


        for csv in csv_files:

            traj_json = self.generate_traj_json(csv)

            gt_config = (
                self.generate_ground_truth_recorder_config(
                    csv
                )
            )

            self.generate_test_json(
                csv,
                traj_json,
                gt_config,
            )


        self.generate_config_json()


# ============================================================
# Main
# ============================================================

def main():

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--input",
        type=Path,
        required=True,
    )

    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Root directory where generated files are written (defaults to script directory)",
    )

    parser.add_argument(
        "--container-root",
        type=str,
        default=None,
        help="Mounted root path inside docker",
    )

    args = parser.parse_args()

    script_dir = Path(__file__).resolve().parent

    output_root = (
        args.output.resolve()
        if args.output
        else script_dir
    )

    generator = TestCaseGenerator(
        input_dir=args.input,
        output_root=output_root,
        container_root=args.container_root,
    )

    generator.generate()

if __name__ == "__main__":
    main()