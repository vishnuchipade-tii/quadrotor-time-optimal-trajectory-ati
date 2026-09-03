# Efficient Time-Optimal Trajectory Generation for Quadrotors 
**<p style="font-size: 20px;"><strong>(Using an Augmented Triple-Integrator)</strong></p>**

A computationally efficient framework for generating **time-optimal trajectories** for quadrotor UAVs using an **augmented triple-integrator model with jerk input**, and systematically comparing its performance against double integrator based trajectory generation methods and a full 6-DOF quadrotor planner (an extdended version of [rpg_time_optimal](https://github.com/uzh-rpg/rpg_time_optimal) planner available at [rpg_time_optimal_extended](https://github.com/vishnuchipade-tii/rpg_time_optimal_extended)).

The framework supports:

- Time-optimal trajectory generation
- Multiple dynamics models
- Multiple planners and solvers
- Benchmarking and comparison experiments
- Automatic report generation (plots, tables, PDF)
- Export of trajectories for downstream tracking controllers

---

# Features

- Augmented Triple Integrator dynamics with jerk control
- Time-optimal trajectory planners
- YAML-based experiment configuration
- Automatic plotting
- Automatic LaTeX table generation
- PDF report generation
- Cached simulation results
- Benchmarking
  - Damped Double Integrator dynamics
  - Full 6-DOF quadrotor model
  - RPG trajectory planner integration

---

# Repository Structure
<details>
<summary><kbd> 📂 <strong>Show Directory Structure</strong></kbd></summary>

```text
.
├── analysis/                   # Post-processing, metrics, trajectory analysis, and report generation
│   ├── report/              # LaTeX/PDF report generation utilities
│   └── trajectory/             # Trajectory extraction and analysis tools
│
├── builders/                   # Factory methods for states, models, targets, planners, and experiments
│
├── config/                     # Vehicle and simulation configuration files
│
├── core/                       # Core abstractions: dynamics, states, targets, experiments, and data structures
│
├── external/                   # Interfaces and support for external tools (e.g., extended RPG planner)
│
├── planners/                    # Planner implementations and planner interfaces
│
├── Runners/                    # Runner implementations for running internal and external planners
│
├── scenario/                   # Experiment scenario definitions (YAML configurations)
│
├── solvers/                    # Optimization and trajectory solver implementations
│
├── trajectory_tracking/        # Support for generating test cases for trajectory tracking using Aurora Simulation Framework
│
├── visualization/              # Visualization tools for trajectories, states, and experiment results
│
├── utils/                      # General-purpose utilities shared across modules
│
├── results/                    # Generated experiment outputs, figures, tables, and reports
│
├── environment.yml             # Conda environment dependencies
│
├── main.py                     # Main experiment execution entry point
│
├── setup_env.sh                # Conda environment installation script
│
├── setup_external_rpg_planner.sh # External RPG planner installation script
│
└── README.md
```
</details>

## Module Overview

The repository is organized into four main layers:

<table>
<tr>
<td width="50%" valign="top">

### 1. Core Layer

Contains the fundamental abstractions used throughout the project:

- Vehicle dynamics models
- State representations
- Targets
- Experiments and simulation data structures

</td>
<td width="50%" valign="top">

### 2. Planning and Tracking Layer

Responsible for generating and following trajectories:

- Planners generate desired trajectories
- Solvers optimize trajectories
- Trajectory tracking support via [Aurora](https://github.com/tiiuae/Aurora) to follow generated trajectories

</td>
</tr>

<tr>
<td width="50%" valign="top">

### 3. Analysis Layer

Processes experiment outputs:

- Trajectory extraction
- Tracking error computation
- Performance metrics
- LaTeX/PDF report generation

</td>
<td width="50%" valign="top">

### 4. Visualization Layer

Provides visualization tools for:

- Trajectory comparisons
- State evolution plots
- 2D/3D trajectory visualization
- Experiment result inspection

</td>
</tr>
</table>


---

# Installation

## 1. Clone the repository

```bash
git clone https://github.com/vishnuchipade-tii/quadrotor-time-optimal-trajectory-ati.git

cd quadrotor-time-optimal-trajectory-ati
```

---

## 2. Install Miniconda (if needed)

This project uses a Conda environment.

If Conda is not already installed, download and install **Miniconda**:

https://docs.conda.io/en/latest/miniconda.html

Verify the installation:

```bash
conda --version
```

---

## 3. Create the environment

From the project root,

```bash
chmod +x setup_env.sh

./setup_env.sh conda [env_name]
```

This script will create the Conda environment, install all required Python packages and configure any project dependencies
- `env_name` is optional, default environment name is `time_optimal_traj_quad`

---

## 4. Activate the environment

```bash
conda activate time_optimal_traj_quad
```

(Replace `time_optimal_traj_quad` with the environment name used when running `setup_env.sh`.)

---

## 5. (Optional) Setup External RPG Time Optimal Planner

Some comparison experiments include the external extended version of RPG time-optimal trajectory planner [rpg_time_optimal_extended](https://github.com/vishnuchipade-tii/rpg_time_optimal_extended) for 6dof motion model. The extended version of the RPG planner is maintained as a separate project and requires its own Python environment.

If you want to run experiments that compare against 6dof motion model based planning using the extended RPG planner, run:

```bash
chmod +x setup_external_rpg_planner.sh

./setup_external_rpg_planner.sh
```

The setup script will:

* Clone the extended RPG time-optimal trajectory planner repository, 
* Run the RPG project's environment setup script
* Create the required Python environment
* Generate the external planner configuration file:

```text
config/external/rpg_time_opt_traj_planner.yaml
```

The default setup uses:

```text
Environment type : conda
Environment name : rpg_env
Install location : ../rpg_time_optimal
```

Custom installation options can be provided:

```bash
./setup_external_rpg.sh <install_dir> <environment_type> <environment_name>
```

Example:

```bash
./setup_external_rpg.sh ../external/rpg conda rpg_env
```

Supported environment types by the RPG planner setup script:

```text
conda
venv
```

After successful setup, scenarios containing an external planner:

```yaml
planner: rpg_time_opt_traj_planner
solver:
  type: external
```

will automatically use the configured RPG planner.

---

# Running Experiments

Experiments are defined using YAML files in the `scenario/` directory.

Some comparison scenarios include external planners. These require the optional external planner setup described above before running.

## Run a scenario set

```bash
python3 main.py scenario/comparison_scenario_set1.yaml
```

## Run multiple scenario sets

```bash
python3 main.py \
    scenario/comparison_scenario_set1.yaml \
    scenario/comparison_scenario_set2.yaml
```

---

## Useful command-line options

```text
usage:

python3 main.py <scenario.yaml> [options]

Options

--output PATH       # Output directory
--scenario NAME     # Run only one scenario from the YAML file
--no-run            # Use cached simulation results
--no-plots          # Disable plotting
--show-plots        # Display plots interactively
```

Examples

Run one scenario only

```bash
python main.py scenario/comparison_scenario_set1.yaml \
    --scenario sphere_target
```

Reuse cached simulations

```bash
python main.py scenario/comparison_scenario_set1.yaml \
    --no-run
```

Disable plotting

```bash
python main.py scenario/comparison_scenario_set1.yaml \
    --no-plots
```

---

# Experiment Configuration

Experiments are specified in YAML.

Example:

```yaml
quad_config: config/quad.yaml

experiments:

  - name: position_target

    mode: compare

    simulation:
      dt: 0.01
      max_duration: 20.0

    ...

    planners:
      - model: 6dof
        planner: rpg_time_opt_traj_planner
        solver:
          type: external
        color: black

      - model: triple_integrator
        planner: time_opt_traj_planner
        solver:
          type: root_finding
          config: config/solvers/root_finding.yaml
        color: blue
```

Two execution modes are supported.

## Comparison mode

Runs multiple planners and compares them against a reference planner.

```yaml
mode: compare
```

---

## Run mode

Runs one or more planners independently without computing comparison metrics.

```yaml
mode: run
```

---

# Supported Dynamics Models

| Model                       | Description                                                             |
|-----------------------------|-------------------------------------------------------------------------|
| Augmented Triple Integrator | Position and velocity and commanded acceleration states with jerk input |
| Augmented Double Integrator | Position, velocity states with acceleration input                       |
| Full 6-DOF*                 | Complete quadrotor rigid-body dynamics                                  |

* requires external extended version of rpg time optimal trajectory planner
---

# Output

Each experiment produces

```text
results/
    vehicle_name/
        scenario_set/
            scenario/
                dynamics/
                    run_metadata.json
                    simulation.json
                    trajectory.csv
          
                scenario_metadata.json
                scenario.pkl
                .pdf (plots)
```

Along with

- CSV metric tables
- LaTeX tables
- PDF report
- trajectory exports


The external extended version of rpg planner saves generated input tracks and corresponding raw results in `scenario/generated_rpg` directory. 

---

## (Optional) trajectory tracking integration

The `trajectory_tracking` module provides utilities for generating trajectory tracking test cases for the external [Aurora](https://github.com/tiiuae/Aurora) testing framework.

This module does **not** perform trajectory tracking simulation or analysis. Instead, it converts generated trajectory results (e.g., CSV trajectories) into the required Aurora test configuration files, including:

- trajectory publisher configurations
- ground truth recorder configurations
- Aurora test definitions
- simulation configuration files

The generated files can then be executed using the external Aurora framework to evaluate trajectory tracking performance.

For installation requirements, configuration details, and usage instructions, see:

[Trajectory Tracking Integration Guide](trajectory_tracking/README.md)

---

# Citation

If you use this software in your research, please cite:

```bibtex
@inproceedings{chipade2026,
  author    = {Vishnu S. Chipade and Filippo Bertoncelli},
  title     = {Efficient Time-optimal Trajectory Generation for Quadrotors Using an Augmented Triple-Integrator},
  booktitle = {Proceedings of the 26th International Conference on Control, Automation and Systems (ICCAS)},
  year      = {2026},
  pages     = {-},
  address   = {Sapporo, Japan},
  publisher = {IEEE}
}
```
---
