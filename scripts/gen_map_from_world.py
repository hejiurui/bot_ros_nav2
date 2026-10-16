#!/usr/bin/env python3
import argparse
import os
import struct

RES = 0.05
X_MIN, X_MAX = -6.20, 6.20
Y_MIN, Y_MAX = -4.70, 4.70

OCCUPIED = [
    (0.0, 4.56, 12.24, 0.12),
    (0.0, -4.56, 12.24, 0.12),
    (-6.06, 0.0, 0.12, 9.24),
    (6.06, 0.0, 0.12, 9.24),
    (-1.8, -2.6, 0.12, 3.8),
    (-1.8, 2.6, 0.12, 3.8),
    (2.0, -3.05, 0.12, 2.9),
    (2.0, 2.45, 0.12, 4.1),
    (-4.5, 2.5, 1.0, 0.8),
    (-4.2, -3.0, 1.6, 0.5),
    (0.5, -2.5, 0.5, 0.5),
    (4.0, 2.8, 1.2, 1.0),
    (4.2, -2.8, 0.6, 0.6),
]

LETHAL, FREE = 0, 254

def raster(path_base):
    width = int(round((X_MAX - X_MIN) / RES))
    height = int(round((Y_MAX - Y_MIN) / RES))
    grid = bytearray([FREE] * (width * height))

    for cx, cy, sx, sy in OCCUPIED:
        x0 = int(round((cx - sx / 2.0 - X_MIN) / RES))
        x1 = int(round((cx + sx / 2.0 - X_MIN) / RES))
        y0 = height - int(round((cy + sy / 2.0 - Y_MIN) / RES))
        y1 = height - int(round((cy - sy / 2.0 - Y_MIN) / RES))
        for row in range(max(y0, 0), min(y1, height)):
            for col in range(max(x0, 0), min(x1, width)):
                grid[row * width + col] = LETHAL

    os.makedirs(os.path.dirname(os.path.abspath(path_base)), exist_ok=True)
    with open(path_base + '.pgm', 'wb') as fh:
        fh.write(b'P5\n%d %d\n255\n' % (width, height))
        fh.write(bytes(grid))

    with open(path_base + '.yaml', 'w', encoding='utf-8') as fh:
        fh.write('image: %s.pgm\n' % os.path.basename(path_base))
        fh.write('mode: trinary\nresolution: %.2f\n' % RES)
        fh.write('origin: [%.2f, %.2f, 0]\n' % (X_MIN, Y_MIN))
        fh.write('negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')

    blocked = grid.count(LETHAL)
    print('[gen_map] %s  (%d x %d px @ %.2f m/px, origin [%.2f, %.2f])'
          % (path_base + '.pgm', width, height, RES, X_MIN, Y_MIN))
    print('[gen_map] 占据 %.1f%%，空闲 %.1f%%'
          % (100.0 * blocked / (width * height),
             100.0 * (1 - blocked / float(width * height))))
    print('[gen_map] 与 worlds/lab.world 的墙体/障碍坐标一一对应；改世界记得同步这里')

def main():
    default = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                           'src', 'navbot_gazebo', 'maps', 'lab_map')
    ap = argparse.ArgumentParser()
    ap.add_argument('-o', '--out', default=default)
    args = ap.parse_args()
    raster(args.out)

if __name__ == '__main__':
    main()
