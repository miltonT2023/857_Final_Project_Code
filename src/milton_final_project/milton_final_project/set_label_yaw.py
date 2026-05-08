import argparse
import math
import os
import sys

from milton_final_project.map_3d_viewer import DEFAULT_MAP_DIR
from milton_final_project.map_3d_viewer import find_latest_map
from milton_final_project.map_3d_viewer import load_labels
from milton_final_project.map_3d_viewer import write_labels


def resolve_map_name(map_dir, map_path):
    if map_path:
        return os.path.basename(os.path.abspath(os.path.expanduser(map_path)))

    latest_map = find_latest_map(map_dir)
    if latest_map is None:
        raise RuntimeError(f'No saved .yaml map found in {map_dir}')
    return os.path.basename(latest_map)


def resolve_yaw(yaw, yaw_deg):
    if yaw is not None and yaw_deg is not None:
        raise ValueError('Use either --yaw or --yaw-deg, not both.')
    if yaw is not None:
        return yaw
    if yaw_deg is not None:
        return math.radians(yaw_deg)
    raise ValueError('Provide --yaw or --yaw-deg.')


def main(args=None):
    parser = argparse.ArgumentParser(
        description='Update one or more saved map labels to a new yaw.',
    )
    parser.add_argument(
        'labels',
        nargs='*',
        default=['robot_start', 'start', 'home', 'original'],
        help='Labels to update. Defaults to start labels.',
    )
    parser.add_argument('--map-dir', default=DEFAULT_MAP_DIR)
    parser.add_argument('--map', default='')
    parser.add_argument('--yaw', type=float, default=None)
    parser.add_argument('--yaw-deg', type=float, default=None)
    parsed_args = parser.parse_args(args=sys.argv[1:] if args is None else args)

    try:
        map_dir = os.path.abspath(os.path.expanduser(parsed_args.map_dir))
        map_name = resolve_map_name(map_dir, parsed_args.map)
        yaw = resolve_yaw(parsed_args.yaw, parsed_args.yaw_deg)
        labels = load_labels(map_dir, map_name)['locations']
        missing = [name for name in parsed_args.labels if name not in labels]
        if missing:
            raise RuntimeError(f'Missing labels: {", ".join(missing)}')

        for label_name in parsed_args.labels:
            labels[label_name]['yaw'] = yaw
        write_labels(map_dir, map_name, labels)
    except (OSError, RuntimeError, ValueError) as exc:
        raise SystemExit(str(exc))

    print(
        f'Updated {", ".join(parsed_args.labels)} in {map_name} '
        f'to yaw={yaw:.6f} rad ({math.degrees(yaw):.1f} deg)'
    )


if __name__ == '__main__':
    main()
