"""Command-line validation for a Unity semantic-map export."""

import argparse
import sys

from handyman_map_tools.semantic_map import SemanticMapError, load, validate


def main(argv=None):
    """Validate an export and return a shell-friendly status code."""
    parser = argparse.ArgumentParser()
    parser.add_argument('input', help='path to semantic_map.json')
    arguments = parser.parse_args(argv)

    try:
        document = load(arguments.input)
        result = validate(document)
    except SemanticMapError as error:
        print(f'ERROR: {error}', file=sys.stderr)
        return 2

    print(f"Environment: {document.get('environment', '<unknown>')}")
    print(f"Rooms: {len(document.get('rooms', []))}")
    print(f"Destinations: {len(document.get('destinations', []))}")
    print(
        'Object spawn candidates: '
        f"{len(document.get('object_spawn_candidates', []))}"
    )
    for warning in result.warnings:
        print(f'WARNING: {warning}')
    for error in result.errors:
        print(f'ERROR: {error}', file=sys.stderr)
    if result.errors:
        return 2
    print('Semantic map validation passed.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
