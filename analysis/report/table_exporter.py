from analysis.report import dataframe_to_latex_table
from pathlib import Path
import pandas as pd

def export_metrics_table(
    metrics_rows,
    output_dir: Path,
    latex_output_dir: Path,
    filename: str,
    csv_filename: str,
    caption: str,
    label: str,
    second_caption: str = None,
):
    """
    Export metrics dataframe as CSV and LaTeX table.

    Returns
    -------
    df : pd.DataFrame
        Generated dataframe

    latex_table : str
        Generated LaTeX table
    """

    if not metrics_rows:
        return pd.DataFrame(), None

    # Dataframe
    df = pd.DataFrame(metrics_rows)

    # CSV export
    csv_file = output_dir / csv_filename

    df.to_csv(csv_file, index=False)

    print(f"Saved metrics CSV: {csv_file}")

    # LaTeX generation
    latex_table = dataframe_to_latex_table(
        df,
        caption=caption,
        label=label,
        second_caption=second_caption
    )

    # Save LaTeX table
    for directory in [output_dir, latex_output_dir]:

        latex_file = directory / filename

        with open(latex_file, "w") as f:
            f.write(latex_table)

        print(f"Saved LaTeX table: {latex_file}")

    return df, latex_table