from dataclasses import dataclass
import pandas as pd

@dataclass
class TrackingSegment:
    start_t: float
    end_t: float
    df: pd.DataFrame