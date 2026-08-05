from dataclasses import dataclass, field
from typing import Dict, List

from analysis.trajectory import TrajectoryComparator
from core.experiment import ExperimentScenario

@dataclass
class ExperimentScenarioComparisonResult:

    reference_name: str

    pair_results: Dict[str, object] = field(
        default_factory=dict
    )

class ExperimentScenarioComparator:

    def __init__(self, group: ExperimentScenario):

        self.group = group

    def compare_to_reference(
        self,
        reference_name: str,
        signals=("p", "v", "omega"),
        special_metrics=("attitude",),
        N=1000
    ):

        # -----------------------------
        # locate reference run
        # -----------------------------

        reference_run = None

        for run in self.group.results:

            if run.name == reference_name:
                reference_run = run
                break

        if reference_run is None:
            raise ValueError(
                f"Reference run '{reference_name}' not found."
            )

        reference_bundle = (
            reference_run.quad_traj_bundle
        )

        output = ExperimentScenarioComparisonResult(
            reference_name=reference_name
        )

        # -----------------------------
        # compare everyone to reference
        # -----------------------------

        for run in self.group.results:

            if run.name == reference_name:
                continue

            # if run.quad_traj_bundle is None:
            #     run.quad_traj_bundle = QuadTrajBundleFactory.from_simulation_result(run.simulation_result)

            # comparator = TrajectoryComparator(
            #     run.quad_traj_bundle,
            #     reference_bundle
            # )
            comparator = TrajectoryComparator(run, reference_run)

            comparator.compute(
                signals=list(signals),
                special_metrics=list(special_metrics),
                N=N
            )

            output.pair_results[
                run.name
            ] = comparator.result

        return output