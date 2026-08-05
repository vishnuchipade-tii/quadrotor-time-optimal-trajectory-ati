from pathlib import Path
import pickle
from typing import List, Tuple, Union
import pandas as pd

#================================================================
#Load CSV files' content into dataframes
#=================================================================
def load_csvs(csvs: Union[str, Path, List[Union[str, Path]]]) -> pd.DataFrame:
  if isinstance(csvs, (str, Path)):
    csvs = [csvs]

  dfs = [pd.read_csv(str(c)) for c in csvs]

  return pd.concat(dfs, ignore_index=True)


#================================================================
#Load Scenario from pkl object
#=================================================================

def load_scenario(output_dir: Path):
  scenario_file = output_dir / "scenario.pkl"

  if not scenario_file.exists():
    raise FileNotFoundError(
      f"No saved scenario found: {scenario_file}"
    )

  with open(scenario_file, "rb") as f:
    scenario = pickle.load(f)

  print(f"[INFO] Loaded scenario from {scenario_file}")

  return scenario

