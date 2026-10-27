#!/usr/bin/env python3
"""Compare native replay outputs without aligning or rescaling their trajectories."""
import argparse
import csv
import json
from pathlib import Path
import struct

import numpy as np


def rows(path):
    with path.open() as stream:
        values = list(csv.reader(stream))
    return (np.array([int(row[0]) for row in values], dtype=np.int64),
            np.array([[float(x) for x in row[1:]] for row in values]))


def clouds(path):
    with path.open('rb') as stream:
        while True:
            header = stream.read(12)
            if not header:
                break
            stamp, count = struct.unpack('<QI', header)
            points = np.frombuffer(stream.read(count*32), dtype='<f4').reshape(count, 8)
            yield stamp, points


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('reference', type=Path)
    parser.add_argument('candidate', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--position-tolerance-m', type=float)
    parser.add_argument('--angle-tolerance-rad', type=float)
    args = parser.parse_args()
    reference = json.loads((args.reference/'summary.json').read_text())
    candidate = json.loads((args.candidate/'summary.json').read_text())
    result = {'same_message_counts': reference['received'] == candidate['received'],
              'same_input_counts': reference['sent'] == candidate['sent']}
    for name in ('odometry', 'imu_odometry', 'tf', 'gravity'):
        xt, x = rows(args.reference/(name+'.csv'))
        yt, y = rows(args.candidate/(name+'.csv'))
        item = {'same_shape': x.shape == y.shape, 'same_timestamps': np.array_equal(xt, yt)}
        item['finite'] = bool(np.isfinite(x).all() and np.isfinite(y).all())
        if x.shape == y.shape:
            item['max_field_difference'] = float(np.max(np.abs(x-y)))
            item['exact_values'] = np.array_equal(x, y)
            if name != 'gravity':
                distance = np.linalg.norm(x[:, :3]-y[:, :3], axis=1)
                q1, q2 = x[:, 3:7], y[:, 3:7].copy()
                q2[np.sum(q1*q2, axis=1) < 0] *= -1
                angle = 4*np.arctan2(np.linalg.norm(q1-q2, axis=1), np.linalg.norm(q1+q2, axis=1))
                item['position_max_m'] = float(distance.max())
                item['position_rms_m'] = float(np.sqrt(np.mean(distance**2)))
                item['angle_max_rad'] = float(angle.max())
        result[name] = item
    fields = reference['body_fields']
    assert fields == candidate['body_fields'] and len(fields) == 8
    from itertools import zip_longest
    item = {'frames': 0, 'same_timestamps': True, 'same_point_counts': True,
            'position_max_m': 0., 'max_field_differences': dict.fromkeys(fields, 0.)}
    for left, right in zip_longest(clouds(args.reference/'body_clouds.bin'), clouds(args.candidate/'body_clouds.bin')):
        assert left is not None and right is not None, 'Different cloud frame counts'
        ts, x = left;tt, y = right
        item['frames'] += 1
        item['same_timestamps'] &= ts == tt
        item['same_point_counts'] &= x.shape == y.shape
        if x.shape != y.shape:
            continue
        diff = np.abs(x.astype(np.float64)-y)
        if len(diff):
            item['position_max_m'] = max(item['position_max_m'], float(np.linalg.norm(diff[:, :3], axis=1).max()))
            for index, field in enumerate(fields):
                item['max_field_differences'][field] = max(item['max_field_differences'][field], float(diff[:, index].max()))
    result['body_clouds'] = item
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))
    assert result['same_message_counts'] and result['same_input_counts']
    for name in ('odometry','imu_odometry','tf','gravity'):
        metric=result[name]
        assert metric['same_shape'] and metric['same_timestamps'] and metric['finite'], name
        if name != 'gravity':
            if args.position_tolerance_m is not None:
                assert metric['position_max_m'] <= args.position_tolerance_m, name
            if args.angle_tolerance_rad is not None:
                assert metric['angle_max_rad'] <= args.angle_tolerance_rad, name
    assert item['same_timestamps'] and item['same_point_counts']
    if args.position_tolerance_m is not None:
        assert item['position_max_m'] <= args.position_tolerance_m, 'body clouds'


if __name__ == '__main__':
    main()
