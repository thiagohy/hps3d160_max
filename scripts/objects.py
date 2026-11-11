#!/usr/bin/env python3
"""Fine-grained scene reading: objects by angular sector + shape analysis."""
import sys, math
sys.path.insert(0, __import__('os').path.dirname(__import__('os').path.abspath(__file__)))
from analyse_scene import load_distances, load_xyz, status

W, H = 160, 60
HFOV, VFOV = 38.0, 16.0   # half angles


def angular_sectors(rows, n=12):
    """Median distance per horizontal sector (angle based)."""
    print('  sector  ang(deg)   valid%   median(mm)  p10   p90')
    for s in range(n):
        a0 = -HFOV + 2 * HFOV * s / n
        a1 = -HFOV + 2 * HFOV * (s + 1) / n
        x0 = int((a0 + HFOV) / (2 * HFOV) * W)
        x1 = int((a1 + HFOV) / (2 * HFOV) * W)
        vals = []
        for y in range(H):
            for x in range(x0, max(x1, x0 + 1)):
                d = rows[y][x]
                if status(d) == 'ok':
                    vals.append(d)
        if not vals:
            print(f'   {s:2d}  [{a0:+6.1f},{a1:+6.1f}]     0.0%         -    -     -')
            continue
        vals.sort()
        p10 = vals[len(vals)//10]; p90 = vals[9*len(vals)//10]
        print(f'   {s:2d}  [{a0:+6.1f},{a1:+6.1f}]  {100*len(vals)/(H*(x1-x0)):6.1f}%  {vals[len(vals)//2]:10d} {p10:5d} {p90:5d}')


def objects(pts, eps=0.045, min_pts=25):
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
        ang = [math.degrees(math.atan2(xs[k], zs[k])) for k in range(len(mem))]
        out.append({'n': len(mem), 'x': (min(xs),max(xs)), 'y': (min(ys),max(ys)), 'z': (min(zs),max(zs)),
                    'd': (min(ds),max(ds)), 'd_mean': sum(ds)/len(ds),
                    'az': (min(ang), max(ang)),
                    'z_mean': sum(zs)/len(zs), 'y_mean': sum(ys)/len(ys)})
    out.sort(key=lambda c: -c['n'])
    return out


def surface_fit(pts):
    """Fit z = a*x + b*y + c (least squares) and report residual."""
    if len(pts) < 30: return None
    n = len(pts)
    Sx = sum(p[0] for p in pts); Sy = sum(p[1] for p in pts); Sz = sum(p[2] for p in pts)
    Sxx = sum(p[0]**2 for p in pts); Syy = sum(p[1]**2 for p in pts); Sxy = sum(p[0]*p[1] for p in pts)
    Sxz = sum(p[0]*p[2] for p in pts); Syz = sum(p[1]*p[2] for p in pts)
    A = [[Sxx, Sxy, Sx], [Sxy, Syy, Sy], [Sx, Sy, n]]
    b = [Sxz, Syz, Sz]
    # gaussian elimination
    for i in range(3):
        piv = max(range(i, 3), key=lambda r: abs(A[r][i]))
        A[i], A[piv] = A[piv], A[i]; b[i], b[piv] = b[piv], b[i]
        if abs(A[i][i]) < 1e-12: return None
        for r in range(i+1, 3):
            f = A[r][i]/A[i][i]
            for c in range(i, 3): A[r][c] -= f*A[i][c]
            b[r] -= f*b[i]
    x = [0]*3
    for i in (2,1,0):
        s = b[i] - sum(A[i][c]*x[c] for c in range(i+1, 3))
        x[i] = s/A[i][i]
    a, bb, c = x
    res = [abs(p[2] - (a*p[0] + bb*p[1] + c)) for p in pts]
    res.sort()
    return {'a': a, 'b': bb, 'c': c, 'res_med': res[len(res)//2], 'res_p90': res[int(len(res)*0.9)]}


def main():
    rows = load_distances(sys.argv[1])
    pts = load_xyz(sys.argv[2])
    print('== angular sectors (horizontal) ==')
    angular_sectors(rows)
    print('\n== objects (cluster eps=45mm) ==')
    for o in objects(pts)[:12]:
        print(f'  {o["n"]:5d} pts  d~{o["d_mean"]:6.0f}mm [{o["d"][0]:4.0f},{o["d"][1]:4.0f}]  '
              f'x[{o["x"][0]:+.3f},{o["x"][1]:+.3f}] y[{o["y"][0]:+.3f},{o["y"][1]:+.3f}] z[{o["z"][0]:.3f},{o["z"][1]:.3f}]  '
              f'az[{o["az"][0]:+6.1f},{o["az"][1]:+6.1f}]deg')
    print('\n== plane fits ==')
    groups = {'near (<0.30m)': [p for p in pts if p[2] < 0.30],
              'mid (0.30-0.60m)': [p for p in pts if 0.30 <= p[2] < 0.60],
              'far (>0.60m)': [p for p in pts if p[2] >= 0.60]}
    for name, g in groups.items():
        f = surface_fit(g)
        if not f:
            print(f'  {name}: n={len(g)} (insufficient)')
            continue
        print(f'  {name}: n={len(g):5d}  z = {f["a"]:+.2f}x {f["b"]:+.2f}y {f["c"]:+.3f}  '
              f'resid med={f["res_med"]*1000:6.1f}mm p90={f["res_p90"]*1000:7.1f}mm')


if __name__ == '__main__':
    main()
