"""Local 2D scan odometry for mapping. Never supplies flight-navigation pose."""
import math

import numpy as np
from scipy.spatial import cKDTree


def rotation(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s, c]])


def transform(points, pose):
    return points @ rotation(pose[2]).T + pose[:2]


def match_scan(reference, current, initial=(0., 0., 0.)):
    """Trimmed robust point-to-line ICP, with overlap and observability gates."""
    if len(reference) < 80 or len(current) < 80:
        raise ValueError('need at least 80 valid scene returns')
    tree = cKDTree(reference)
    _, neighbours = tree.query(reference, k=7)
    local = reference[neighbours]
    centred = local - local.mean(axis=1, keepdims=True)
    values, vectors = np.linalg.eigh(np.einsum('nki,nkj->nij', centred, centred))
    normals = vectors[:, :, 0]
    line = values[:, 0] < .15 * np.maximum(values[:, 1], 1e-9)
    pose = np.array(initial, dtype=float)
    converged = False
    for _ in range(25):
        rotated = current @ rotation(pose[2]).T
        moved = rotated + pose[:2]
        distance, nearest = tree.query(moved)
        valid = (distance < .35) & line[nearest]
        if valid.sum() < 60 or valid.mean() < .5:
            raise ValueError('insufficient overlap; mapping held')
        p, n, q = rotated[valid], normals[nearest[valid]], reference[nearest[valid]]
        error = np.einsum('ij,ij->i', n, moved[valid] - q)
        cutoff = max(.012, float(np.quantile(abs(error), .8)))
        keep = abs(error) <= cutoff
        p, n, error = p[keep], n[keep], error[keep]
        A = np.column_stack((n[:, 0], n[:, 1], -n[:, 0]*p[:, 1] + n[:, 1]*p[:, 0]))
        normalised = A / np.maximum(np.linalg.norm(A, axis=0), 1e-9)
        eigenvalues = np.linalg.eigvalsh(normalised.T @ normalised)
        if eigenvalues[0] < .002 * eigenvalues[-1]:
            raise ValueError('scene cannot constrain translation and yaw; need corners or nonparallel walls')
        weight = np.minimum(1., .025 / np.maximum(abs(error), 1e-8))
        delta = np.linalg.lstsq(A * np.sqrt(weight[:, None]), -error*np.sqrt(weight), rcond=None)[0]
        if np.linalg.norm(delta[:2]) > .25 or abs(delta[2]) > math.radians(10):
            raise ValueError('registration step too large; mapping held')
        pose += delta
        if np.linalg.norm(delta[:2]) < .0002 and abs(delta[2]) < .0001:
            converged = True
            break
    if not converged:
        raise ValueError('scan matching did not converge; mapping held')
    moved = transform(current, pose)
    distances, nearest = tree.query(moved)
    use = (distances < .15) & line[nearest]
    if use.mean() < .55:
        raise ValueError('too few consistent returns; mapping held')
    residual = np.einsum('ij,ij->i', normals[nearest[use]], moved[use]-reference[nearest[use]])
    rmse = float(np.sqrt(np.mean(np.minimum(residual**2, .15**2))))
    if rmse > .04:
        raise ValueError('scan alignment error exceeds 40 mm; mapping held')
    return pose, {'rmse_m': rmse, 'overlap': float(use.mean()), 'returns': int(len(current))}


class ScanOdometry:
    def __init__(self):
        self.reference = None
        self.reference_pose = np.zeros(3)
        self.relative = np.zeros(3)
        self.pose = np.zeros(3)
        self.stamp = None
        self.input_stamp = None
        self.recovering = False

    def update(self, points, stamp):
        points = np.asarray(points, dtype=float)
        if points.ndim != 2 or points.shape[1] != 2 or not np.isfinite(points).all() or len(points) < 80:
            raise ValueError('need at least 80 finite scene points')
        if not math.isfinite(stamp) or stamp <= 0:
            raise ValueError('invalid capture timestamp')
        # Limit registration work without changing the published full scan.
        if len(points) > 720:
            points = points[::math.ceil(len(points)/720)]
        if self.reference is None:
            # Self-registration proves the initial scene constrains all 3 DOF.
            _, quality = match_scan(points, points)
            self.reference = points.copy()
            self.stamp = stamp
            self.input_stamp = stamp
            return self.pose.copy(), quality
        input_dt = stamp - self.input_stamp
        if input_dt <= 0:
            raise ValueError('capture clock reset; restart mapping to establish a new local origin')
        self.input_stamp = stamp
        if input_dt > 1.:
            self.recovering = True
            raise ValueError('capture gap; map held until consecutive scans revalidate the previous scene')
        dt = min(.3, stamp - self.stamp)
        self.recovering = self.recovering or stamp-self.stamp > 1.
        relative, quality = match_scan(self.reference, points, self.relative)
        xy = self.reference_pose[:2] + rotation(self.reference_pose[2]) @ relative[:2]
        pose = np.r_[xy, self.reference_pose[2] + relative[2]]
        if self.recovering and (quality['overlap'] < .8 or quality['rmse_m'] > .02 or
                               np.linalg.norm(pose[:2]-self.pose[:2]) > .15 or
                               abs(pose[2]-self.pose[2]) > math.radians(3)):
            raise ValueError('lost scene is not revalidated; restart mapping at a new origin')
        if np.linalg.norm(pose[:2] - self.pose[:2]) > 2*dt + .03 or abs(pose[2]-self.pose[2]) > math.radians(90)*dt + .03:
            raise ValueError('motion exceeds scan-matching bound; mapping held')
        self.pose, self.relative, self.stamp = pose, relative, stamp
        self.recovering = False
        if np.linalg.norm(relative[:2]) > .3 or abs(relative[2]) > .2:
            self.reference, self.reference_pose, self.relative = points.copy(), pose.copy(), np.zeros(3)
        return pose.copy(), quality
