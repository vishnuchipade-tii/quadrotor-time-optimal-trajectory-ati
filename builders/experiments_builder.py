from core.type import DynamicsType
from core.config import PlannerConfig
from core.type import RunnerType
from core.simulation import (SimMetadata)

from builders import DynamicsBuilder
from builders import StateBuilder
from builders import TargetBuilder
from builders import PlanningRequestBuilder
from builders import PlannerBuilder

from core.experiment import ExperimentCase, ExperimentScenario

from core.type import PlannerType
from external.rpg_time_opt_traj_planner import RPGTimeOptimalTrajectoryPlannerBackend

import yaml
from pathlib import Path


def get_simplified_model_params(quad_cfg):
  cfg = quad_cfg["models"]["simplified"]

  params = {
    "cd": cfg["cd"],
    "ca": cfg["ca"],
    "use_gravity": cfg["use_gravity"],
  }

  bounds = {
    "j_max": cfg["bounds"]["j_max"]
  }

  bounds["a_max"] = bounds["j_max"] / params["ca"]

  return params, bounds

def build_experiments(config_path, quad_cfg):
  with open(config_path, "r") as f:
    cfg = yaml.safe_load(f)

  scenarios = []
  scenario_set_name = Path(config_path).stem

  for exp_cfg in cfg["experiments"]:

    sim_metadata = SimMetadata(
      dt=exp_cfg["simulation"]["dt"],
      max_duration=exp_cfg["simulation"]["max_duration"]
    )

    target = TargetBuilder.build(exp_cfg["target"], model="augmented_triple_integrator")

    initial_state = StateBuilder.build({
      **exp_cfg["initial_state"],
      "type": exp_cfg["initial_state"]["type"]
    })

    scenario = ExperimentScenario(
      name=exp_cfg["name"],
      mode=exp_cfg.get("mode", "run"),
      scenario_set_name=scenario_set_name,
      initial_state=initial_state,
      target=target,
      target_type=target.target_type
    )

    for experiment_planner_spec in exp_cfg["planners"]:

      model_name = experiment_planner_spec["model"]

      planner_spec = experiment_planner_spec["planner"]
      solver_spec = experiment_planner_spec["solver"]

      planner_type = PlannerType.from_string(
        planner_spec["type"]
      )

      planner_config = None

      if "config" in planner_spec:
        planner_config = PlannerConfig.from_config(
          planner_spec.get("config", {})
        )

      color = experiment_planner_spec["color"]

      if model_name in ["augmented_triple_integrator", "augmented_double_integrator"]:
        params, input_bounds = get_simplified_model_params(quad_cfg)

      elif model_name == "6dof":
        params = quad_cfg["models"]["6dof"]
        input_bounds = None
      else:
        params = dict()
        input_bounds = None

      dynamics_model = DynamicsBuilder.build(
        DynamicsType.from_string(model_name),
        params
      )

      if planner_type is PlannerType.RPG_TIME_OPT_TRAJ:
        backend = RPGTimeOptimalTrajectoryPlannerBackend.from_yaml(
                "config/external/rpg_time_opt_traj_planner.yaml"
            )

        planner = PlannerBuilder.build(PlannerType.RPG_TIME_OPT_TRAJ, backend=backend)

        case = ExperimentCase(
          name="RPG",
          runner_type=RunnerType.EXTERNAL_RPG,

          dynamics_model=dynamics_model,
          planner=planner,
          params=params,
          quad_config=quad_cfg,

          initial_state=None,
          target=None,

          sim_metadata=sim_metadata,

          color=color,

          experiment_config=exp_cfg
        )

        scenario.cases.append(case)

        continue

      target = TargetBuilder.build(exp_cfg["target"], model=model_name)

      initial_state = StateBuilder.build({
        **exp_cfg["initial_state"],
        "type": model_name
      })

      planning_request = PlanningRequestBuilder.build(
        initial_state=initial_state.as_vector(),
        dynamics=dynamics_model,
        target=target,
        input_bounds=input_bounds,
        planner_spec=experiment_planner_spec,
      )

      planner = PlannerBuilder.build(PlannerType.TIME_OPT_TRAJ, planning_request=planning_request, planner_config=planner_config)

      planner_id = planner.planner_id

      solver_id = planner.solver_id

      case = ExperimentCase(
        name=f"{model_name}_{planner_id}",
        dynamics_model=dynamics_model,
        planner=planner,
        params=params,
        quad_config=quad_cfg,
        initial_state=initial_state,
        target=target,
        sim_metadata=sim_metadata,
        color=color
      )

      scenario.cases.append(case)

    scenarios.append(scenario)

  return scenarios