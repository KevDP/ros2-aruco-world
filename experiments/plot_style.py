"""One look for every figure the experiments produce: dark background, light text."""

from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")  # files only, no window

import matplotlib.pyplot as plt  # noqa: E402  (the backend is chosen first)

BACKGROUND = "#0f172a"
PANEL = "#1e293b"
TEXT = "#e2e8f0"
MUTED = "#94a3b8"
CYAN = "#22d3ee"
AMBER = "#fbbf24"
VIOLET = "#c4b5fd"
EMERALD = "#34d399"

plt.rcParams.update(
    {
        "figure.facecolor": BACKGROUND,
        "axes.facecolor": BACKGROUND,
        "savefig.facecolor": BACKGROUND,
        "axes.edgecolor": MUTED,
        "axes.labelcolor": TEXT,
        "axes.titlecolor": TEXT,
        "xtick.color": MUTED,
        "ytick.color": MUTED,
        "text.color": TEXT,
        "grid.color": PANEL,
        "legend.facecolor": PANEL,
        "legend.edgecolor": PANEL,
        "font.family": "monospace",
        "font.size": 10,
        "axes.titlesize": 12,
    }
)


def save(figure: plt.Figure, path: Path) -> None:
    """Write the figure as a PNG sized for a blog column, and free it."""
    figure.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(figure)
