#!/usr/bin/env python3
"""Plot Veloxity vs upstream-C bags from the upstream follower experiment."""

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("TkAgg")
import matplotlib.pyplot as plt

from plot_quadx_firmware_compare import (
    plot_error_magnitudes,
    plot_errors,
    plot_estimate_delta,
    plot_overview,
    plot_truth_minus_rust,
    read_bag,
)


def relabel_backends(fig) -> None:
    """Replace experiment-1 Rust/C labels with Veloxity/C labels."""

    def replace(text: str) -> str:
        text = text.replace("Rust", "Veloxity").replace("rust", "Veloxity")
        if text == "c" or text.startswith("c "):
            text = "C" + text[1:]
        return text

    for text in fig.texts:
        text.set_text(replace(text.get_text()))
    for axis in fig.axes:
        axis.set_title(replace(axis.get_title()))
        for artist in axis.lines:
            artist.set_label(replace(artist.get_label()))
        legend = axis.get_legend()
        if legend is not None:
            for text in legend.get_texts():
                text.set_text(replace(text.get_text()))


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--veloxity-bag",
        type=Path,
        default=(
            root
            / "takeoff_logs"
            / "quadx_upstream_backend_compare_veloxity_accel_quat_fix_repeat"
        ),
    )
    parser.add_argument(
        "--c-bag",
        type=Path,
        default=root / "takeoff_logs" / "quadx_upstream_backend_compare_c",
    )
    args = parser.parse_args()

    veloxity = read_bag(args.veloxity_bag)
    c_backend = read_bag(args.c_bag)

    figures = (
        plot_overview(veloxity, c_backend, None),
        plot_errors(veloxity, c_backend, None),
        plot_error_magnitudes(veloxity, c_backend, None),
        plot_estimate_delta(veloxity, c_backend, None),
        plot_truth_minus_rust(veloxity, None),
    )
    for fig in figures:
        relabel_backends(fig)
    plt.show()


if __name__ == "__main__":
    main()
