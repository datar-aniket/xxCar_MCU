"""Host-side full 3D magnetometer ellipsoid calibration.

The GUI owns the raw sample set.  The flight controller receives only a
finished candidate and independently validates its numerical safety before
offering a live preview.  NumPy keeps the large least-squares work off the MCU
and lets the fitter weight spherical regions equally instead of weighting the
orientations where the operator happened to move slowly.
"""

from dataclasses import dataclass
import math

import numpy as np


MIN_SAMPLES = 5000
MAX_SAMPLES = 10000
AZIMUTH_BINS = 24
Z_BINS = 12                       # equal-area when binned directly in z
TOTAL_BINS = AZIMUTH_BINS * Z_BINS
MIN_COVERAGE = 0.55


class FitError(ValueError):
    """A measured dataset cannot produce a calibration safe to store."""

    def __init__(self, reason: str, **metrics):
        super().__init__(reason)
        self.reason = reason
        self.metrics = metrics


@dataclass
class MagFit:
    offset: np.ndarray
    matrix: np.ndarray
    field: float
    rms: float
    maximum_error: float
    condition: float
    used: int
    rejected: int
    occupied_bins: int
    coverage: float
    octants: int
    accepted: np.ndarray
    corrected: np.ndarray
    radial_error: np.ndarray

    def stage_values(self) -> list[float]:
        """Values in the MCU protocol's offset/diag/offdiag/field order."""
        m = self.matrix
        return [*self.offset.tolist(), m[0, 0], m[1, 1], m[2, 2],
                m[0, 1], m[0, 2], m[1, 2], self.field]


def apply_calibration(samples, offset, matrix):
    samples = np.asarray(samples, dtype=np.float64)
    return (samples - np.asarray(offset, dtype=np.float64)) @ \
        np.asarray(matrix, dtype=np.float64).T


def _design(u):
    x, y, z = u.T
    return np.column_stack((x*x, y*y, z*z, 2*x*y, 2*x*z, 2*y*z,
                            x, y, z))


def _algebraic(samples, weights=None):
    mean = samples.mean(axis=0)
    centered = samples - mean
    scale = math.sqrt(float(np.mean(np.sum(centered * centered, axis=1))))
    if not np.isfinite(scale) or scale <= 0.05:
        raise FitError("sample cloud is degenerate")
    u = centered / scale
    design = _design(u)
    target = np.ones(len(samples))
    if weights is not None:
        root = np.sqrt(np.asarray(weights, dtype=np.float64))
        design = design * root[:, None]
        target = target * root
    solution, _, rank, _ = np.linalg.lstsq(design, target, rcond=None)
    if rank < 9:
        raise FitError("ellipsoid fit is singular", rank=int(rank))
    a = np.array(((solution[0], solution[3], solution[4]),
                  (solution[3], solution[1], solution[5]),
                  (solution[4], solution[5], solution[2])))
    try:
        center = -0.5 * np.linalg.solve(a, solution[6:9])
    except np.linalg.LinAlgError as exc:
        raise FitError("ellipsoid center is singular") from exc
    beta = 1.0 + float(center @ a @ center)
    if not np.isfinite(beta) or beta <= 0:
        raise FitError("quadratic is not a closed ellipsoid")
    q = a / beta
    eigenvalue, eigenvector = np.linalg.eigh(q)
    if not np.all(np.isfinite(eigenvalue)) or np.min(eigenvalue) <= 0:
        raise FitError("quadratic is not positive definite")
    field = scale * float(np.prod(eigenvalue) ** (-1.0 / 6.0))
    root = (eigenvector * np.sqrt(eigenvalue)) @ eigenvector.T
    matrix = field * root / scale
    offset = mean + scale * center
    return offset, matrix, field


def _directions(samples, offset, matrix):
    corrected = apply_calibration(samples, offset, matrix)
    norms = np.linalg.norm(corrected, axis=1)
    valid = np.isfinite(norms) & (norms > 1e-9)
    direction = np.zeros_like(corrected)
    direction[valid] = corrected[valid] / norms[valid, None]
    return corrected, norms, direction, valid


def _bin_indices(direction):
    azimuth = np.arctan2(direction[:, 1], direction[:, 0])
    azimuth_index = np.floor((azimuth + np.pi) *
                             (AZIMUTH_BINS / (2*np.pi))).astype(int)
    azimuth_index = np.clip(azimuth_index, 0, AZIMUTH_BINS - 1)
    z_index = np.floor((direction[:, 2] + 1.0) * (Z_BINS / 2)).astype(int)
    z_index = np.clip(z_index, 0, Z_BINS - 1)
    return z_index * AZIMUTH_BINS + azimuth_index


def _uniform_weights(direction, mask):
    bins = _bin_indices(direction)
    count = np.bincount(bins[mask], minlength=TOTAL_BINS)
    weights = np.zeros(len(direction), dtype=np.float64)
    weights[mask] = 1.0 / count[bins[mask]]
    return weights, bins, count


def _robust_mask(norms, field, base_mask):
    error = np.abs(norms - field)
    values = error[base_mask]
    median = float(np.median(values))
    mad = float(np.median(np.abs(values - median)))
    threshold = max(0.015, median + 4.0 * 1.4826 * mad)
    return base_mask & (error <= threshold)


def _cost(samples, offset, matrix, field, weights):
    corrected = apply_calibration(samples, offset, matrix)
    residual = np.linalg.norm(corrected, axis=1) - field
    return float(np.sum(weights * residual * residual))


def _refine(samples, offset, matrix, field, weights):
    """Geometric Gauss-Newton fit with a damped, bounded line search."""
    for _ in range(12):
        v = samples - offset
        w = v @ matrix.T
        norm = np.linalg.norm(w, axis=1)
        valid = (weights > 0) & np.isfinite(norm) & (norm > 1e-9)
        if np.count_nonzero(valid) < 9:
            break
        vv, ww, nn = v[valid], w[valid], norm[valid]
        unit = ww / nn[:, None]
        jacobian = np.empty((len(vv), 9), dtype=np.float64)
        jacobian[:, :3] = -(unit @ matrix)
        jacobian[:, 3] = unit[:, 0] * vv[:, 0]
        jacobian[:, 4] = unit[:, 1] * vv[:, 1]
        jacobian[:, 5] = unit[:, 2] * vv[:, 2]
        jacobian[:, 6] = unit[:, 0] * vv[:, 1] + unit[:, 1] * vv[:, 0]
        jacobian[:, 7] = unit[:, 0] * vv[:, 2] + unit[:, 2] * vv[:, 0]
        jacobian[:, 8] = unit[:, 1] * vv[:, 2] + unit[:, 2] * vv[:, 1]
        residual = nn - field
        weight = weights[valid]
        normal = jacobian.T @ (weight[:, None] * jacobian)
        right = -(jacobian.T @ (weight * residual))
        normal += np.diag(1e-6 * (np.diag(normal) + 1.0))
        try:
            delta = np.linalg.solve(normal, right)
        except np.linalg.LinAlgError:
            break
        old_cost = _cost(samples, offset, matrix, field, weights)
        accepted = False
        for step in (1.0, 0.5, 0.25, 0.125, 0.0625, 0.03125):
            trial_offset = offset + step * delta[:3]
            p = np.array((matrix[0, 0], matrix[1, 1], matrix[2, 2],
                          matrix[0, 1], matrix[0, 2], matrix[1, 2]))
            p += step * delta[3:]
            trial_matrix = np.array(((p[0], p[3], p[4]),
                                     (p[3], p[1], p[5]),
                                     (p[4], p[5], p[2])))
            eigenvalue = np.linalg.eigvalsh(trial_matrix)
            if np.min(eigenvalue) <= 0.1 or np.max(eigenvalue) >= 10.0:
                continue
            trial_cost = _cost(samples, trial_offset, trial_matrix,
                               field, weights)
            if np.isfinite(trial_cost) and trial_cost < old_cost:
                offset, matrix = trial_offset, trial_matrix
                accepted = True
                break
        if not accepted:
            break
    return offset, matrix


def fit_ellipsoid(samples) -> MagFit:
    samples = np.asarray(samples, dtype=np.float64)
    if samples.ndim != 2 or samples.shape[1] != 3:
        raise FitError("samples must have shape N×3")
    finite = np.all(np.isfinite(samples), axis=1)
    plausible = np.linalg.norm(samples, axis=1) < 4.0
    samples = samples[finite & plausible]
    if len(samples) < MIN_SAMPLES:
        raise FitError("need more samples", have=len(samples),
                       need=MIN_SAMPLES)

    offset, matrix, field = _algebraic(samples)
    for _ in range(3):
        corrected, norms, direction, valid = _directions(samples, offset,
                                                          matrix)
        accepted = _robust_mask(norms, field, valid)
        weights, _, _ = _uniform_weights(direction, accepted)
        if np.count_nonzero(accepted) < int(0.8 * MIN_SAMPLES):
            raise FitError("too many magnetic outliers",
                           used=int(np.count_nonzero(accepted)))
        offset, matrix, field = _algebraic(samples[accepted],
                                            weights[accepted])
        corrected, norms, direction, valid = _directions(samples, offset,
                                                          matrix)
        accepted = _robust_mask(norms, field, valid)
        weights, _, _ = _uniform_weights(direction, accepted)
        offset, matrix = _refine(samples, offset, matrix, field, weights)

    corrected, norms, direction, valid = _directions(samples, offset, matrix)
    accepted = _robust_mask(norms, field, valid)
    weights, bins, counts = _uniform_weights(direction, accepted)
    occupied = int(np.count_nonzero(counts))
    coverage = occupied / TOTAL_BINS
    octant_index = ((direction[:, 0] >= 0).astype(int) |
                    ((direction[:, 1] >= 0).astype(int) << 1) |
                    ((direction[:, 2] >= 0).astype(int) << 2))
    octants = 0
    for value in np.unique(octant_index[accepted]):
        octants |= 1 << int(value)
    minimum = direction[accepted].min(axis=0)
    maximum = direction[accepted].max(axis=0)
    if octants != 0xff or np.any(minimum > -0.55) or np.any(maximum < 0.55):
        raise FitError("poor 3D coverage", octants=octants,
                       coverage=coverage)
    if coverage < MIN_COVERAGE:
        raise FitError("too many uncovered spherical regions",
                       coverage=coverage, occupied=occupied,
                       total=TOTAL_BINS)

    eigenvalue = np.linalg.eigvalsh(matrix)
    condition = float(eigenvalue[-1] / eigenvalue[0])
    radial_error = norms - field
    used_error = radial_error[accepted]
    rms = float(np.sqrt(np.mean(used_error * used_error)))
    maximum_error = float(np.max(np.abs(used_error)))
    if not 0.15 <= field <= 0.80:
        raise FitError("field strength out of range", field=field)
    if np.any(np.abs(offset) > 1.5):
        raise FitError("hard-iron offset out of range", offset=offset.tolist())
    if eigenvalue[0] < 0.25 or eigenvalue[-1] > 4.0 or condition > 4.0:
        raise FitError("soft-iron matrix out of range", condition=condition,
                       eigenvalues=eigenvalue.tolist())
    if rms > min(0.030, 0.07 * field) or maximum_error > 0.100:
        raise FitError("fit residual too high", rms=rms,
                       maximum=maximum_error)

    return MagFit(offset=offset, matrix=matrix, field=float(field), rms=rms,
                  maximum_error=maximum_error, condition=condition,
                  used=int(np.count_nonzero(accepted)),
                  rejected=int(len(samples) - np.count_nonzero(accepted)),
                  occupied_bins=occupied, coverage=coverage, octants=octants,
                  accepted=accepted, corrected=corrected,
                  radial_error=radial_error)


def validate_corrected(samples, field):
    """Quality metrics for fresh samples returned by the staged MCU preview."""
    samples = np.asarray(samples, dtype=np.float64)
    if samples.ndim != 2 or samples.shape[1] != 3 or len(samples) == 0:
        raise FitError("corrected samples must have shape N×3")
    if not np.all(np.isfinite(samples)) or not np.isfinite(field):
        raise FitError("corrected samples are not finite")
    norms = np.linalg.norm(samples, axis=1)
    error = norms - float(field)
    return {
        "count": len(samples),
        "rms": float(np.sqrt(np.mean(error * error))),
        "maximum": float(np.max(np.abs(error))),
        "mean": float(np.mean(norms)),
    }
