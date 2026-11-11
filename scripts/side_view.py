#!/usr/bin/env python3
"""Side view + depth profile of an HPS-3D160 capture (depth vs height)."""
import sys, math
sys.path.insert(0, __import__('os').path.dirname(__import__('os').path.abspath(__file__)))
from analyse_scene import load_distances, load_xyz, status

RAMP = '.:-=+*#%@'


def side_view(pts, cols=110, rows=34):
    ok = [p for p in pts if 0 < p[3] < 65530]
    if not ok:
        return '(none)'
    zs = [p[2] for p in ok]
    ys = [p[1] for p in ok]
    z0, z1 = min(zs), max(zs)
    y0, y1 = min(ys), max(ys)
    grid = [[0] * cols for _ in range(rows)]
    for p in ok:
        cx = int((p[2] - z0) / ((z1 - z0) or 1) * (cols - 1))
        cy = int((p[1] - y0) / ((y1 - y0) or 1) * (rows - 1))
        grid[cy][cx] += 1
    peak = max(max(r) for r in grid) or 1
    out = []
    for r in reversed(grid):
        out.append(''.join(' ' if v == 0 else RAMP[min(len(RAMP)-1, int(v/peak*(len(RAMP)-1)))] for v in r))
    return '\n'.join(out)


def depth_profile(pts, bins=40):
    ok = [p for p in pts if 0 < p[3] < 65530]
    if not ok:
        return
    zs = [p[2] for p in ok]
    z0, z1 = min(zs), max(zs)
    step = (z1 - z0) / bins
    print(f'  depth(m)     n     height y: min/med/max      note')
    for i in range(bins):
        a = z0 + i * step
        b = a + step
        sel = [p for p in ok if a <= p[2] < b]
        if not sel:
            continue
        ys = sorted(p[1] for p in sel)
        med = ys[len(ys) // 2]
        note = ''
        if ys[-1] - ys[0] < 0.25:
            note = 'flat (vertical surface / table edge)'
        print(f'  {a:5.2f}-{b:5.2f} {len(sel):5d}   {ys[0]:+6.2f}/{med:+6.2f}/{ys[-1]:+6.2f}   {note}')


def horizontal_extent(pts, bins=30):
    ok = [p for p in pts if 0 < p[3] < 65530]
    if not ok:
        return
    zs = [p[2] for p in ok]
    z0, z1 = min(zs), max(zs)
    step = (z1 - z0) / bins
    print('\n  depth(m)   x: min/med/max (m)   width(m)')
    for i in range(bins):
        a = z0 + i * step
        b = a + step
        sel = [p for p in ok if a <= p[2] < b]
        if not sel:
            continue
        xs = sorted(p[0] for p in sel)
        med = xs[len(xs) // 2]
        print(f'  {a:5.2f}-{b:5.2f}  {xs[0]:+7.2f}/{med:+7.2f}/{xs[-1]:+7.2f}   {xs[-1]-xs[0]:6.2f}')


def main():
    dist = sys.argv[1]
    xyz = sys.argv[2]
    rows = load_distances(dist)
    pts = load_xyz(xyz)
    print('== side view (depth ->, height y up) ==')
    print('   '.rstrip())
    print(side_view(pts))
    print('\n== depth profile ==')
    depth_profile(pts)
    horizontal_extent(pts)


if __name__ == '__main__':
    main()
