import argparse
import os
import shutil
import sys

import cv2
import numpy as np


DEFAULT_MAP_DIR = '/home/nvidia/857_Final_Project_Code/maps'
LEGACY_LABELS_FILE_NAME = 'map_labels.yaml'


def find_latest_map(map_dir):
    if not os.path.isdir(map_dir):
        return None

    yaml_paths = [
        os.path.join(map_dir, name)
        for name in os.listdir(map_dir)
        if name.endswith('.yaml')
        and not name.endswith('.labels.yaml')
        and name != LEGACY_LABELS_FILE_NAME
    ]
    if not yaml_paths:
        return None

    return max(yaml_paths, key=os.path.getmtime)


def parse_simple_yaml(path):
    fields = {}
    with open(path, 'r', encoding='utf-8') as yaml_file:
        for line in yaml_file:
            if ':' not in line:
                continue
            key, value = line.split(':', 1)
            fields[key.strip()] = value.strip()
    return fields


def image_path_for_yaml(yaml_path):
    fields = parse_simple_yaml(yaml_path)
    image_name = fields.get('image')
    if not image_name:
        raise ValueError(f'Map yaml does not contain an image field: {yaml_path}')

    if os.path.isabs(image_name):
        return image_name
    return os.path.join(os.path.dirname(yaml_path), image_name)


def read_pgm(path):
    with open(path, 'rb') as pgm_file:
        magic = pgm_file.readline().strip()
        if magic != b'P5':
            raise ValueError(f'Only binary PGM P5 maps are supported: {path}')

        line = pgm_file.readline()
        while line.startswith(b'#'):
            line = pgm_file.readline()

        width, height = [int(part) for part in line.split()]
        max_value = int(pgm_file.readline().strip())
        if max_value != 255:
            raise ValueError(f'Unsupported PGM max value {max_value}: {path}')

        pixels = pgm_file.read(width * height)
        if len(pixels) != width * height:
            raise ValueError(f'PGM data is incomplete: {path}')

    image = np.frombuffer(pixels, dtype=np.uint8).reshape((height, width)).copy()
    return image


def write_pgm(path, image):
    height, width = image.shape
    with open(path, 'wb') as pgm_file:
        header = f'P5\n# CREATOR: milton_final_project filter_saved_map\n{width} {height}\n255\n'
        pgm_file.write(header.encode('ascii'))
        pgm_file.write(image.astype(np.uint8).tobytes())


def remove_small_components(occupied, min_component_cells):
    if min_component_cells <= 1:
        return occupied

    component_count, labels, stats, _ = cv2.connectedComponentsWithStats(
        occupied,
        connectivity=8,
    )
    cleaned = np.zeros_like(occupied)
    for component_id in range(1, component_count):
        area = stats[component_id, cv2.CC_STAT_AREA]
        if area >= min_component_cells:
            cleaned[labels == component_id] = 1
    return cleaned


def close_wall_gaps(occupied, close_kernel_cells):
    if close_kernel_cells <= 1:
        return occupied

    kernel_cells = close_kernel_cells
    if kernel_cells % 2 == 0:
        kernel_cells += 1

    horizontal_kernel = np.ones((1, kernel_cells), dtype=np.uint8)
    vertical_kernel = np.ones((kernel_cells, 1), dtype=np.uint8)
    horizontal = cv2.morphologyEx(
        occupied,
        cv2.MORPH_CLOSE,
        horizontal_kernel,
    )
    vertical = cv2.morphologyEx(
        occupied,
        cv2.MORPH_CLOSE,
        vertical_kernel,
    )
    return np.maximum.reduce([occupied, horizontal, vertical])


def filter_map_image(
    image,
    min_component_cells,
    close_kernel_cells,
    free_open_kernel_cells,
    occupied_pixel_threshold,
):
    occupied = (image <= occupied_pixel_threshold).astype(np.uint8)
    free = (image >= 250).astype(np.uint8)
    unknown = image == 205

    cleaned = close_wall_gaps(occupied, close_kernel_cells)
    cleaned = remove_small_components(cleaned, min_component_cells)

    filtered = image.copy()
    if free_open_kernel_cells > 1:
        kernel_cells = free_open_kernel_cells
        if kernel_cells % 2 == 0:
            kernel_cells += 1
        free_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (kernel_cells, kernel_cells),
        )
        opened_free = cv2.morphologyEx(free, cv2.MORPH_OPEN, free_kernel)
        removed_free_rays = (free == 1) & (opened_free == 0) & (cleaned == 0)
        filtered[removed_free_rays] = 205

    removed_noise = (occupied == 1) & (cleaned == 0)
    added_wall_fill = (occupied == 0) & (cleaned == 1)

    filtered[removed_noise] = 205
    filtered[added_wall_fill & unknown] = 0
    filtered[added_wall_fill & ~unknown] = 0
    return filtered


def output_paths(yaml_path, image_path, suffix, overwrite):
    if overwrite:
        return yaml_path, image_path

    base_yaml, yaml_ext = os.path.splitext(yaml_path)
    base_image, image_ext = os.path.splitext(image_path)
    return f'{base_yaml}{suffix}{yaml_ext}', f'{base_image}{suffix}{image_ext}'


def copy_yaml_with_image(yaml_path, output_yaml_path, output_image_path):
    output_image_name = os.path.basename(output_image_path)
    lines = []
    with open(yaml_path, 'r', encoding='utf-8') as yaml_file:
        for line in yaml_file:
            if line.startswith('image:'):
                lines.append(f'image: {output_image_name}\n')
            else:
                lines.append(line)

    with open(output_yaml_path, 'w', encoding='utf-8') as yaml_file:
        yaml_file.writelines(lines)


def backup_file(path):
    backup_path = f'{path}.raw'
    if not os.path.exists(backup_path):
        shutil.copy2(path, backup_path)
    return backup_path


def filter_saved_map(
    yaml_path,
    overwrite,
    suffix,
    min_component_cells,
    close_kernel_cells,
    free_open_kernel_cells,
    occupied_pixel_threshold,
):
    yaml_path = os.path.abspath(os.path.expanduser(yaml_path))
    image_path = image_path_for_yaml(yaml_path)
    image = read_pgm(image_path)
    filtered = filter_map_image(
        image,
        min_component_cells,
        close_kernel_cells,
        free_open_kernel_cells,
        occupied_pixel_threshold,
    )
    output_yaml_path, output_image_path = output_paths(
        yaml_path,
        image_path,
        suffix,
        overwrite,
    )

    if overwrite:
        backup_yaml = backup_file(yaml_path)
        backup_image = backup_file(image_path)
        copy_yaml_with_image(yaml_path, output_yaml_path, output_image_path)
        write_pgm(output_image_path, filtered)
        return output_yaml_path, output_image_path, backup_yaml, backup_image

    copy_yaml_with_image(yaml_path, output_yaml_path, output_image_path)
    write_pgm(output_image_path, filtered)
    return output_yaml_path, output_image_path, None, None


def main(args=None):
    parser = argparse.ArgumentParser(
        description='Clean a saved Nav2/SLAM Toolbox PGM occupancy map.',
    )
    parser.add_argument(
        '--map',
        default='',
        help='Map yaml path. Defaults to latest map in --map-dir.',
    )
    parser.add_argument('--map-dir', default=DEFAULT_MAP_DIR)
    parser.add_argument(
        '--overwrite',
        action='store_true',
        help='Replace the input yaml/pgm and keep .raw backups.',
    )
    parser.add_argument(
        '--suffix',
        default='_filtered',
        help='Suffix for filtered output when not using --overwrite.',
    )
    parser.add_argument(
        '--min-component-cells',
        type=int,
        default=8,
        help='Remove occupied blobs smaller than this many pixels.',
    )
    parser.add_argument(
        '--close-kernel-cells',
        type=int,
        default=5,
        help='Fill small horizontal/vertical wall gaps up to this kernel size.',
    )
    parser.add_argument(
        '--free-open-kernel-cells',
        type=int,
        default=3,
        help='Remove thin free-space ray spikes up to this kernel size.',
    )
    parser.add_argument(
        '--occupied-pixel-threshold',
        type=int,
        default=25,
        help='PGM pixels at or below this value are treated as occupied.',
    )
    parsed_args = parser.parse_args(args=sys.argv[1:] if args is None else args)

    map_dir = os.path.abspath(os.path.expanduser(parsed_args.map_dir))
    yaml_path = parsed_args.map
    if yaml_path:
        yaml_path = os.path.abspath(os.path.expanduser(yaml_path))
    else:
        yaml_path = find_latest_map(map_dir)
        if yaml_path is None:
            raise SystemExit(f'No saved .yaml map found in {map_dir}')

    try:
        output_yaml, output_image, backup_yaml, backup_image = filter_saved_map(
            yaml_path,
            parsed_args.overwrite,
            parsed_args.suffix,
            max(1, parsed_args.min_component_cells),
            max(1, parsed_args.close_kernel_cells),
            max(1, parsed_args.free_open_kernel_cells),
            max(0, min(255, parsed_args.occupied_pixel_threshold)),
        )
    except (OSError, ValueError) as exc:
        raise SystemExit(str(exc))

    print(f'Filtered map yaml: {output_yaml}')
    print(f'Filtered map image: {output_image}')
    if backup_yaml and backup_image:
        print(f'Raw backup yaml: {backup_yaml}')
        print(f'Raw backup image: {backup_image}')


if __name__ == '__main__':
    main()
