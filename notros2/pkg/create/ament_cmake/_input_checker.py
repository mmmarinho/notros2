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