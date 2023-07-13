import argparse
import os
import pathlib


def _check_common_inputs(path: pathlib.Path, args: argparse.Namespace):
    if path is None:
        raise ValueError('Path must not be None.')
    if args is None:
        raise ValueError('args must not be None.')
    if not os.path.isdir(path):
        raise ValueError(f'the path={path} does not exist.')


def _parse_ament_dependencies_for_cmake(args: argparse.Namespace) -> str:
    ament_dependencies_str = ""
    if args.ament_dependencies is not None:
        for ament_dependency in args.ament_dependencies:
            ament_dependencies_str = ament_dependencies_str + f'    {ament_dependency}\n'
    return ament_dependencies_str