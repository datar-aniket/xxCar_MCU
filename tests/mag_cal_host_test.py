#!/usr/bin/env python3
"""Deterministic checks for the host-side magnetometer fitter."""

import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "tools"))

from mag_cal import FitError, apply_calibration, fit_ellipsoid, validate_corrected


def synthetic(seed=7, count=5400):
    rng = np.random.default_rng(seed)
    direction = rng.normal(size=(count, 3))
    direction /= np.linalg.norm(direction, axis=1)[:, None]
    # Deliberately dwell near one orientation; bin weighting must prevent bias.
    direction[:900] = np.array((1.0, 0.0, 0.0)) + rng.normal(
        0.0, 0.08, (900, 3))
    direction[:900] /= np.linalg.norm(direction[:900], axis=1)[:, None]
    field = 0.47
    offset = np.array((0.13, -0.08, 0.055))
    matrix = np.array(((1.18, 0.07, -0.03),
                       (0.07, 0.84, 0.04),
                       (-0.03, 0.04, 1.03)))
    raw = direction * field @ np.linalg.inv(matrix).T + offset
    raw += rng.normal(0.0, 0.0015, raw.shape)
    # Sparse gross interference should be rejected, not distort the fit.
    raw[::173] += rng.normal(0.0, 0.12, raw[::173].shape)
    return raw, offset, matrix, field


def main():
    raw, expected_offset, expected_matrix, expected_field = synthetic()
    fit = fit_ellipsoid(raw)
    assert np.max(np.abs(fit.offset - expected_offset)) < 0.004
    assert np.max(np.abs(fit.matrix - expected_matrix)) < 0.035
    assert abs(fit.field - expected_field) < 0.012
    assert fit.coverage > 0.90
    assert fit.rejected > 10
    assert fit.rms < 0.004

    fresh_raw, _, _, _ = synthetic(seed=19, count=5200)
    keep = np.ones(len(fresh_raw), dtype=bool)
    keep[::173] = False
    fresh = apply_calibration(fresh_raw[keep][:600], fit.offset, fit.matrix)
    quality = validate_corrected(fresh, fit.field)
    assert quality["count"] == 600
    assert quality["rms"] < 0.004

    hemisphere = raw[raw[:, 2] > expected_offset[2]]
    hemisphere = np.tile(hemisphere, (4, 1))[:5200]
    try:
        fit_ellipsoid(hemisphere)
    except FitError:
        pass
    else:
        raise AssertionError("incomplete 3D coverage was accepted")

    try:
        fit_ellipsoid(raw[:4999])
    except FitError as exc:
        assert exc.reason == "need more samples"
    else:
        raise AssertionError("undersized dataset was accepted")

    print("mag_cal host: recovery, outlier and coverage checks verified - OK")


if __name__ == "__main__":
    main()
