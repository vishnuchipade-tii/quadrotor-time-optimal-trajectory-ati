import matplotlib as mpl
import matplotlib.pyplot as plt
from pathlib import Path

DEFAULT_STYLE = {
    "text.usetex": True,
    "font.family": "serif",

    "font.size": 24,

    "axes.labelsize": 24,
    "axes.titlesize": 26,

    "xtick.labelsize": 20,
    "ytick.labelsize": 20,

    "legend.fontsize": 20,
}

def configure_matplotlib():
    """Configure global matplotlib defaults for this project."""
    mpl.rcParams.update(DEFAULT_STYLE)


#Set style from .mplstyle file

_STYLE_DIR = Path(__file__).parent / "styles"

def use_style(name: str = "paper") -> None:
    plt.style.use(_STYLE_DIR / f"{name}.mplstyle")
