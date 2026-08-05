from utils.strings import clean_string

def clean_dataframe_for_latex(df):
    """
    Cleans:
    - column names
    - ALL cell values (including case_study, algorithm, etc.)
    """

    df = df.copy()

    # -------------------------
    # clean column names
    # -------------------------
    df.columns = [clean_string(c) for c in df.columns]

    # -------------------------
    # clean all cell values
    # -------------------------
    for col in df.columns:
        df[col] = df[col].map(clean_string)

    return df


def wide_to_long(df, id_cols=("case study", "model")):

    value_cols = [c for c in df.columns if c not in id_cols]

    long_df = df.melt(
        id_vars=list(id_cols),
        value_vars=value_cols,
        var_name="metric",
        value_name="value"
    )

    return long_df