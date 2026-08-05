from typing import Union
from pathlib import Path
import yaml

DEFAULT_ATTITUDE = [1.0, 0.0, 0.0, 0.0]
DEFAULT_OMEGA = [0.0, 0.0, 0.0]

def experiment_to_rpg_track(exp_cfg: dict) -> dict:
    """
    Convert one experiment definition into an RPG track yaml dict.

    Parameters
    ----------
    exp_cfg : dict
        One experiment entry from comparison_scenario_set1.yaml

    Returns
    -------
    dict
        RPG track configuration
    """

    initial_cfg = exp_cfg["initial_state"]
    target_cfg = exp_cfg["target"]

    track = {
        "gates": [],
        "initial": {
            "position": initial_cfg["pos"],
            "attitude": DEFAULT_ATTITUDE,
            "velocity": initial_cfg["vel"],
            "omega": DEFAULT_OMEGA,
        }
    }

    # --------------------------------------------------
    # Position target
    # --------------------------------------------------

    if target_cfg["type"] == "position":
        track["end"] = {
            "type": "position",
            "position": target_cfg["pos"],
        }

    # --------------------------------------------------
    # Hemisphere target
    # --------------------------------------------------

    elif target_cfg["type"] == "sphere":

        track["end"] = {
            "type": "sphere",
            "center": target_cfg["center"],
            "radius": float(target_cfg["radius"]),
        }

    # --------------------------------------------------
    # Fixed endpoint target
    # --------------------------------------------------

    elif target_cfg["type"] == "state":

        track["end"] = {
            "type": "state",
            "position": target_cfg["pos"],
            "velocity": target_cfg["vel"],
        }

    else:

        raise ValueError(
            f"Unsupported target type: "
            f"{target_cfg['type']}"
        )

    return track


def save_rpg_track(
    exp_cfg: dict,
    output_file: Union[str, Path]
):
    """
    Build and save RPG track yaml.
    """

    output_file = Path(output_file)

    track_dict = experiment_to_rpg_track(exp_cfg)

    output_file.parent.mkdir(
        parents=True,
        exist_ok=True
    )

    with open(output_file, "w") as f:

        yaml.safe_dump(
            track_dict,
            f,
            sort_keys=False
        )

    return output_file

def generate_rpg_tracks(
    experiment_yaml,
    output_dir
):
    with open(experiment_yaml) as f:
        cfg = yaml.safe_load(f)

    output_dir = Path(output_dir)

    for exp_cfg in cfg["experiments"]:

        save_rpg_track(
            exp_cfg,
            output_dir /
            exp_cfg["name"] /
            "track.yaml"
        )