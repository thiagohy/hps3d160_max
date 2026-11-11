#!/usr/bin/env python3
"""Compares two HPS-3D160 captures to check for the presence of a person.

Usage: compare_presence.py <dir_sem_pessoa> <dir_com_pessoa>
Each directory must contain frames.csv and frame_*.distances.csv produced by
`hps3d160_max_node --record-dir`.
"""
import csv
import os
import sys


def load(dirname):
    csv_path = os.path.join(dirname, "frames.csv")
    rows = list(csv.DictReader(open(csv_path)))
    dist_files = sorted(f for f in os.listdir(dirname) if f.endswith(".distances.csv"))
    matrices = []
    for f in dist_files:
        rows_d = []
        with open(os.path.join(dirname, f)) as fh:
            for line in fh:
                line = line.strip()
                if line:
                    rows_d.append([int(v) for v in line.split(",")])
        matrices.append(rows_d)
    return rows, matrices


def stats(matrices):
    n = len(matrices)
    if n == 0:
        return {}
    h, w = len(matrices[0]), len(matrices[0][0])
    valid = 0
    near = 0
    total = 0
    mean_d = 0.0
    for m in matrices:
        for row in m:
            for d in row:
                total += 1
                if 0 < d < 60000:
                    valid += 1
                    mean_d += d
                    if d < 2000:
                        near += 1
    return {
        "frames": n,
        "valid_pct": 100.0 * valid / total,
        "near_2m_pct": 100.0 * near / total,
        "mean_dist_mm": mean_d / valid if valid else 0,
        "avg_from_csv": float(rows_mean(matrices)),
    }


def rows_mean(matrices):
    return 0.0


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return 1
    a_dir, b_dir = sys.argv[1], sys.argv[2]
    ra, ma = load(a_dir)
    rb, mb = load(b_dir)
    sa, sb = stats(ma), stats(mb)

    def mean_dist(rows):
        vals = [float(r["distance_avg_mm"]) for r in rows if r.get("distance_avg_mm")]
        return sum(vals) / len(vals) if vals else 0.0

    def mean_valid(rows):
        vals = [float(r["valid_points"]) for r in rows if r.get("valid_points")]
        return sum(vals) / len(vals) if vals else 0

    print(f"{'':22s} {'A (sem pessoa)':>16s} {'B (com pessoa)':>16s} {'delta':>10s}")
    print(f"{'frames':22s} {len(ma):16d} {len(mb):16d}")
    print(f"{'distancia media (mm)':22s} {mean_dist(ra):16.1f} {mean_dist(rb):16.1f} "
          f"{mean_dist(rb)-mean_dist(ra):+10.1f}")
    print(f"{'pixels validos (media)':22s} {mean_valid(ra):16.0f} {mean_valid(rb):16.0f} "
          f"{mean_valid(rb)-mean_valid(ra):+10.0f}")
    print(f"{'% validos':22s} {sa.get('valid_pct',0):16.1f} {sb.get('valid_pct',0):16.1f} "
          f"{sb.get('valid_pct',0)-sa.get('valid_pct',0):+10.1f}")
    print(f"{'% pontos a < 2 m':22s} {sa.get('near_2m_pct',0):16.1f} {sb.get('near_2m_pct',0):16.1f} "
          f"{sb.get('near_2m_pct',0)-sa.get('near_2m_pct',0):+10.1f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
