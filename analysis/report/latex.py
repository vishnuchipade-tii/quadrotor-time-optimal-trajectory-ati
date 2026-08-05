from __future__ import annotations

import numpy as np
import pandas as pd

from .labels import apply_label_map
from .labels import LABEL_MAP
from .utils import clean_dataframe_for_latex, wide_to_long


def dataframe_to_latex_table(df, float_fmt="%.3f", na_rep="--", caption=None, label=None, second_caption=None):
    """
        Converts a pandas DataFrame to a latex table.

        Parameters:
        df (DataFrame): The DataFrame to convert.
        float_fmt (str): The format to use for floating point numbers. Defaults to "%.3f".
        na_rep (str): The representation to use for missing values. Defaults to "--".
        caption (str): The caption to use for the table. Defaults to None.
        label (str): The label to use for the table. Defaults to None.
        second_caption (str): The second caption to use for the table, without numbering. Defaults to None.

        Returns:
        str: The latex table as a string.
    """

    df = df.copy()

    df = clean_dataframe_for_latex(df)

    df = wide_to_long(df)

    df["metric"] = df["metric"].map(
        lambda x: apply_label_map(x, LABEL_MAP)
    )

    if "metric" not in df.columns:
        raise ValueError("DataFrame must contain a 'metric' column")

    # ----------------------------------------------------
    # Pivot (same as before)
    # ----------------------------------------------------
    metric_order = df["metric"].drop_duplicates().tolist()

    df["metric"] = pd.Categorical(
        df["metric"],
        categories=metric_order,
        ordered=True
    )

    pivot = df.pivot_table(
        index="metric",
        columns=["case study", "model"],
        values="value",
        aggfunc="first",
        sort=False
    )

    #pivot = pivot.sort_index(axis=1, level=[0, 1])

    case_studies = pivot.columns.levels[0]
    algorithms = pivot.columns.levels[1]

    # ----------------------------------------------------
    # Begin LaTeX table
    # ----------------------------------------------------
    lines = []

    lines.append("\\begin{table*}[ht]")
    lines.append("\\centering")

    if caption:
        lines.append(f"\\caption{{{caption}}}")

    if label:
        lines.append(f"\\label{{{label}}}")

    # column layout: metric + all subcolumns
    n_subcols = len(pivot.columns)
    col_spec = "l" + "c" * n_subcols

    lines.append(f"\\begin{{tabular}}{{{col_spec}}}")
    lines.append("\\toprule")

    # ----------------------------------------------------
    # HEADER ROW 1: Case studies
    # ----------------------------------------------------
    header1 = [" "]
    for cs in case_studies:
        header1.append(f"\\multicolumn{{{len(algorithms)}}}{{c}}{{{cs.capitalize()}}}")

    lines.append(" & ".join(header1) + " \\\\")

    # ----------------------------------------------------
    # HEADER ROW 2: cmidrules
    # ----------------------------------------------------
    span_blocks = []
    start = 2

    for cs in case_studies:
        end = start + len(algorithms) - 1
        span_blocks.append(f"\\cmidrule(lr){{{start}-{end}}}")
        start = end + 1

    lines.extend(span_blocks)

    # ----------------------------------------------------
    # HEADER ROW 3: algorithms
    # ----------------------------------------------------
    header2 = ["Metric"]
    for _ in case_studies:
        header2.extend(list(algorithms))

    lines.append(" & ".join(header2) + " \\\\")
    lines.append("\\midrule")

    # ----------------------------------------------------
    # DATA ROWS
    # ----------------------------------------------------
    for metric in pivot.index:

        row = [metric]

        for cs in case_studies:
            for alg in algorithms:

                if (cs, alg) in pivot.columns:
                    val = pivot.loc[metric, (cs, alg)]
                else:
                    val = np.nan

                if pd.isna(val):
                    row.append(na_rep)
                else:
                    row.append(float_fmt % val)

        lines.append(" & ".join(row) + " \\\\")

    # ----------------------------------------------------
    # End table
    # ----------------------------------------------------
    lines.append("\\bottomrule")
    lines.append("\\end{tabular}")

    # Add second caption
    if second_caption:
        lines.append(r"\caption*{" + second_caption + "}")

    lines.append("\\end{table*}")

    return "\n".join(lines)


def dataframe_to_latex_table_using_pivot(df, float_fmt="%.3f", na_rep="--"):
  """
  Case Study → grouped columns
  Algorithm → sub-columns
  Metrics → rows
  """

  df = df.copy()

  # ----------------------------------------------------
  # Ensure long format
  # ----------------------------------------------------
  if "metric" not in df.columns:
    raise ValueError("DataFrame must contain a 'metric' column")

  # ----------------------------------------------------
  # Pivot:
  # rows = metric
  # columns = (case study, algorithm)
  # ----------------------------------------------------
  pivot = df.pivot_table(
    index="metric",
    columns=["case study", "model"],
    values="value",
    aggfunc="first"
  )

  # ----------------------------------------------------
  # Sort for consistent paper output
  # ----------------------------------------------------
  pivot = pivot.sort_index(axis=1, level=[0, 1])

  # ----------------------------------------------------
  # LaTeX export
  # ----------------------------------------------------
  latex = pivot.to_latex(
    multirow=True,
    multicolumn=True,
    float_format=float_fmt,
    na_rep=na_rep
  )

  return latex
