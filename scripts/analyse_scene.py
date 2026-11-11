#!/usr/bin/env python3
"""Describe the scene captured by an HPS-3D160-U.

Usage: analyse_scene.py <distances.csv> [points.xyz]
"""
import sys, math, random
from collections import Counter

INVALID, LOW_AMP, SAT, ADC = 65530, 65300, 65400, 65500
RAMP = '.:-=+*#%@'


def load_distances(path):
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line:
                rows.append([int(v) for v in line.split(',')])
    return rows


def load_xyz(path):
    out = []
    with open(path) as f:
        for line in f:
            p = line.split()
            if len(p) == 4:
                out.append(tuple(map(float, p)))
    return out


def status(d):
    if d == 0: return 'zero'
    if d == LOW_AMP: return 'low_amp'
    if d == SAT: return 'sat'
    if d == ADC: return 'adc'
    if d >= INVALID: return 'invalid'
    return 'ok'


def render_front(rows, cols=100, rows_out=32):
    H, W = len(rows), len(rows[0])
    vals = [d for r in rows for d in r if status(d) == 'ok']
    if not vals: return '(none)'
    lo, hi = min(vals), max(vals)
    out = []
    for ry in range(rows_out):
        y = min(H - 1, int(ry * H / rows_out))
        line = []
        for rx in range(cols):
            x = min(W - 1, int(rx * W / cols))
            s = status(rows[y][x])
            if s == 'ok':
                t = 0.0 if hi == lo else (rows[y][x] - lo) / (hi - lo)
                line.append(RAMP[min(len(RAMP)-1, int(t*(len(RAMP)-1)))])
            elif s in ('sat', 'adc'):
                line.append('#')      # saturated / overflow
            elif s == 'low_amp':
                line.append('w')      # weak signal
            else:
                line.append('.')      # invalid / no return
        out.append(''.join(line))
    return '\n'.join(out)


def render_top(pts, cols=100, rows_out=32):
    """Top view: horizontal x (right) vs depth z (down = farther)."""
    ok = [p for p in pts if 0 < p[3] < INVALID]
    if not ok: return '(none)'
    xs = [p[0] for p in ok]; zs = [p[2] for p in ok]
    x0, x1 = min(xs), max(xs); z0, z1 = min(zs), max(zs)
    grid = [[0]*cols for _ in range(rows_out)]
    for p in ok:
        cx = int((p[0]-x0)/((x1-x0) or 1)*(cols-1))
        cz = int((p[2]-z0)/((z1-z0) or 1)*(rows_out-1))
        grid[cz][cx] += 1
    peak = max(max(r) for r in grid) or 1
    out = []
    for r in reversed(grid):        # far at top
        line = ''.join(' ' if v == 0 else RAMP[min(len(RAMP)-1, int(v/peak*(len(RAMP)-1)))] for v in r)
        out.append(line)
    return '\n'.join(out)


def ransac_plane(pts, thresh=0.015, iters=6000):
    if len(pts) < 100: return None
    random.seed(11)
    best = None
    sample = random.sample(pts, min(600, len(pts)))
    for _ in range(iters):
        p, q, r = random.sample(sample, 3)
        v1 = [q[i]-p[i] for i in range(3)]
        v2 = [r[i]-p[i] for i in range(3)]
        n = [v1[1]*v2[2]-v1[2]*v2[1], v1[2]*v2[0]-v1[0]*v2[2], v1[0]*v2[1]-v1[1]*v2[0]]
        norm = math.sqrt(sum(c*c for c in n)) or 1.0
        n = [c/norm for c in n]
        d = -(n[0]*p[0]+n[1]*p[1]+n[2]*p[2])
        inl = [q2 for q2 in pts if abs(n[0]*q2[0]+n[1]*q2[1]+n[2]*q2[2]+d) < thresh]
        if best is None or len(inl) > len(best[1]):
            best = (n, inl, d)
    n, inl, d = best
    tilt = math.degrees(math.acos(min(1.0, abs(n[2]))))
    az = math.degrees(math.atan2(n[0], n[2]))
    return {'normal': n, 'distance': abs(d), 'inliers': len(inl),
            'ratio': len(inl)/len(pts), 'tilt_deg': tilt, 'azimuth_deg': az}


def column_profile(rows, step=8):
    W = len(rows[0]); H = len(rows)
    print('  x(px)  x(m)    valid%   median(mm)  min(mm)  max(mm)')
    for x0 in range(0, W, step):
        vals = [rows[y][x0] for y in range(H) if status(rows[y][x0]) == 'ok']
        if not vals:
            print(f'  {x0:4d}   ----        0%         -       -       -')
            continue
        vals_sorted = sorted(vals)
        med = vals_sorted[len(vals)//2]
        ang = (x0 - W/2) / (W/2) * 38.0     # horizontal half FoV 38 deg
        print(f'  {x0:4d}  {ang:+6.1f}  {100*len(vals)/H:6.1f}%  {med:10d} {vals_sorted[0]:8d} {vals_sorted[-1]:8d}')


def row_profile(rows, step=6):
    H = len(rows); W = len(rows[0])
    print('  y(px)  y(m)    valid%   median(mm)')
    for y0 in range(0, H, step):
        vals = [rows[y0][x] for x in range(W) if status(rows[y0][x]) == 'ok']
        if not vals:
            print(f'  {y0:4d}   ----       0%          -')
            continue
        vals_sorted = sorted(vals)
        ang = (y0 - H/2) / (H/2) * 16.0     # vertical half FoV 16 deg
        print(f'  {y0:4d}  {ang:+6.1f}  {100*len(vals)/W:6.1f}%  {vals_sorted[len(vals)//2]:8d}')


def cluster(pts, eps=0.15, min_pts=60):
    grid = {}
    for i, p in enumerate(pts):
        grid.setdefault((int(p[0]//eps), int(p[1]//eps), int(p[2]//eps)), []).append(i)
    seen, clusters = set(), []
    for i in range(len(pts)):
        if i in seen: continue
        seen.add(i); q = [i]; mem = []
        while q:
            j = q.pop(); mem.append(j)
            pj = pts[j]; k = (int(pj[0]//eps), int(pj[1]//eps), int(pj[2]//eps))
            for dx in (-1,0,1):
                for dy in (-1,0,1):
                    for dz in (-1,0,1):
                        for m in grid.get((k[0]+dx,k[1]+dy,k[2]+dz), []):
                            if m in seen: continue
                            pm = pts[m]
                            if sum((pj[t]-pm[t])**2 for t in range(3)) <= eps*eps:
                                seen.add(m); q.append(m)
        if len(mem) >= min_pts: clusters.append(mem)
    out = []
    for mem in clusters:
        xs=[pts[i][0] for i in mem]; ys=[pts[i][1] for i in mem]; zs=[pts[i][2] for i in mem]; ds=[pts[i][3] for i in mem]
        out.append({'n': len(mem), 'x': (min(xs),max(xs)), 'y': (min(ys),max(ys)),
                    'z': (min(zs),max(zs)), 'd': (min(ds),max(ds)),
                    'z_mean': sum(zs)/len(zs), 'd_mean': sum(ds)/len(ds)})
    out.sort(key=lambda c: -c['n'])
    return out


def main():
    dist_path = sys.argv[1]
    xyz_path = sys.argv[2] if len(sys.argv) > 2 else None
    rows = load_distances(dist_path)
    pts = load_xyz(xyz_path) if xyz_path else []
    H, W = len(rows), len(rows[0])
    hist = Counter(status(d) for r in rows for d in r)
    total = sum(hist.values())
    print(f'== {dist_path}  ({W}x{H} = {total} px) ==')
    for k, v in hist.most_common():
        print(f'   {k:8s} {v:6d}  {100*v/total:5.1f}%')
    ok = sorted(d for r in rows for d in r if status(d) == 'ok')
    if ok:
        print(f'   valid distance: min={ok[0]} p25={ok[len(ok)//4]} median={ok[len(ok)//2]} '
              f'p75={ok[3*len(ok)//4]} max={ok[-1]} mm')
    if pts:
        print(f'   points={len(pts)}  x[{min(p[0] for p in pts):.2f},{max(p[0] for p in pts):.2f}] '
              f'y[{min(p[1] for p in pts):.2f},{max(p[1] for p in pts):.2f}] '
              f'z[{min(p[2] for p in pts):.2f},{max(p[2] for p in pts):.2f}] m')
    print('\n== front view (x ->, y v) ==')
    print('   legend: "."=invalid  "#"=saturated/overflow  "w"=weak  ramp .=near .. @=far ==')
    print(render_front(rows))
    if pts:
        print('\n== top view (x ->, depth z down=far) ==')
        print(render_top(pts))
    pl = ransac_plane(pts) if pts else None
    if pl:
        print(f'\n== dominant plane ==\n   normal=({pl["normal"][0]:.3f},{pl["normal"][1]:.3f},{pl["normal"][2]:.3f})'
              f'  dist={pl["distance"]:.2f}m  tilt={pl["tilt_deg"]:.1f}deg  az={pl["azimuth_deg"]:.1f}deg'
              f'  inliers={pl["inliers"]} ({100*pl["ratio"]:.1f}%)')
    print('\n== column profile ==')
    column_profile(rows)
    print('\n== row profile ==')
    row_profile(rows)
    if pts:
        print('\n== clusters ==')
        for c in cluster(pts)[:8]:
            print(f'   {c["n"]:5d} pts x[{c["x"][0]:6.2f},{c["x"][1]:6.2f}] y[{c["y"][0]:6.2f},{c["y"][1]:6.2f}] '
                  f'z[{c["z"][0]:5.2f},{c["z"][1]:5.2f}] d[{c["d"][0]:4.0f},{c["d"][1]:4.0f}] mean_z={c["z_mean"]:.2f}')


if __name__ == '__main__':
    main()
