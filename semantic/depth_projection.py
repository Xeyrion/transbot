"""Auditable depth selection. All modes output a visible surface sample, not a centre."""
import numpy as np
from camera_geometry import undistort


def project_depth(depth, k, matrix, bbox, mode='center', instance_mask=None,
                  distortion=None, max_range=8.):
    x, y, bw, bh = bbox
    h, w = depth.shape
    stats = dict(method=mode)

    def reject(reason):
        return dict(point=None, reason=reason, quality=stats)

    if mode not in ('center', 'surface', 'mask-surface'):
        raise ValueError('Unknown projection method: '+mode)
    if mode == 'mask-surface':
        if instance_mask is None:
            return reject('missing_instance_mask')
        if instance_mask.shape != depth.shape:
            raise ValueError('Instance mask is not on the RGB depth grid')
        u0, u1, v0, v1 = max(0, x), min(w, x+bw), max(0, y), min(h, y+bh)
    else:
        u0, u1 = max(0, x+bw//4), min(w, x+3*bw//4)
        v0, v1 = max(0, y+bh//4), min(h, y+3*bh//4)
    if u1 <= u0 or v1 <= v0 or not (0 <= x+bw/2. < w and 0 <= y+bh/2. < h):
        return reject('invalid_roi')
    patch = depth[v0:v1, u0:u1]
    region = (instance_mask[v0:v1, u0:u1] > 0 if mode == 'mask-surface'
              else np.ones(patch.shape, dtype=bool))
    valid_mask = region & np.isfinite(patch) & (patch > .2) & (patch < max_range)
    valid = patch[valid_mask]
    fraction = float(valid.size)/max(1, int(region.sum()))
    stats.update(valid_fraction=fraction, valid_pixels=int(valid.size), region_pixels=int(region.sum()))
    if valid.size < 10 or fraction < .25:
        return reject('insufficient_valid_depth')
    z = float(np.median(valid))
    mad = float(np.median(np.abs(valid-z)))
    stats.update(depth_mad_m=mad)
    if mode == 'center':
        if mad > max(.1, .1*z):
            return reject('mixed_depth')
        selected = valid_mask & (np.abs(patch-z) <= max(.03, 3*1.4826*mad))
        rows, cols = np.nonzero(selected)
        if len(rows) < 10 or float(len(rows))/valid.size < .7:
            return reject('insufficient_surface_support')
        # Keep the legacy baseline exactly reproducible.
        cu, cv = float(np.median(cols))+u0, float(np.median(rows))+v0
    else:
        # Separate depth layers first; never turn an arbitrary hole into depth.
        # Connected support prevents scattered pixels at the same range voting
        # as one surface. Competing layers with similar support are rejected.
        import cv2
        hist, edges = np.histogram(valid, bins=np.arange(.2, max_range+.08001, .08))
        smooth = np.convolve(hist, [.25, .5, .25], mode='same')
        candidates = []
        visited = []
        for bin_index in np.argsort(-smooth):
            centre = (edges[bin_index]+edges[bin_index+1])/2
            if smooth[bin_index] <= 0 or len(visited) >= 5:
                break
            if any(abs(centre-old) < .24 for old in visited):
                continue
            visited.append(centre)
            band = valid_mask & (np.abs(patch-centre) <= max(.08, .04*centre))
            n, components = cv2.connectedComponents(band.astype(np.uint8), connectivity=4)
            if n <= 1:  # Includes background; no bridging across holes.
                continue
            counts = np.bincount(components.ravel());counts[0] = 0
            winner = int(counts.argmax());count = int(counts[winner])
            if count < max(10, .20*valid.size):
                continue
            pixels = components == winner
            values = patch[pixels]
            surface_z = float(np.median(values))
            if any(abs(surface_z-c['z']) < .18 for c in candidates):
                continue
            candidates.append(dict(z=surface_z, count=count, pixels=pixels))
        if not candidates:
            return reject('no_connected_surface')
        candidates.sort(key=lambda c: -c['count'])
        best = candidates[0]
        if len(candidates) > 1 and candidates[1]['count'] >= .8*best['count']:
            stats['competing_depths_m'] = [c['z'] for c in candidates[:2]]
            return reject('ambiguous_surfaces')
        selected = best['pixels']
        rows, cols = np.nonzero(selected)
        values = patch[selected]
        z = best['z'];mad = float(np.median(np.abs(values-z)))
        # Pick a real supported pixel close to the median ray and depth.
        cost = ((cols-np.median(cols))/max(1., patch.shape[1]))**2
        cost += ((rows-np.median(rows))/max(1., patch.shape[0]))**2
        cost += ((values-z)/max(.02, 1.4826*mad))**2
        j = int(cost.argmin());cu, cv = float(cols[j]+u0), float(rows[j]+v0)
        z = float(values[j])
        stats.update(depth_mad_m=mad, selected_fraction=float(len(rows))/valid.size,
                     competing_surfaces=len(candidates)-1)
    rx, ry = undistort((cu-k[2])/k[0], (cv-k[5])/k[4], distortion)
    if not np.isfinite(rx) or not np.isfinite(ry):
        return reject('invalid_camera_ray')
    p = [float(rx)*z, float(ry)*z, z]
    point = [sum(matrix[i][j]*p[j] for j in range(3))+matrix[i][3] for i in range(3)]+[z]
    stats.update(support_pixels=len(rows), sigma_m=max(.02, 1.4826*mad),
                 pixel_uv=[cu, cv],
                 uncertainty_scope='depth_dispersion_only; excludes calibration and pose error',
                 position_semantics='visible_surface_sample; not object centre')
    return dict(point=point, reason=None, quality=stats)
