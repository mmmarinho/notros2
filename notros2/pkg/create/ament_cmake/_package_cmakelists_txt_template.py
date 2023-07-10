import os
import textwrap
import argparse
import pathlib

from ._add_node import get_cmake_for_nodes
from ._add_library import get_cmake_for_library


def create_cmakelists_txt_from_template(path: pathlib.Path, args: argparse.Namespace) -> None:
    if path is None:
        raise ValueError('Path must not be None.')
    if args is None:
        raise ValueError('args must not be None.')
    if not os.path.isdir(path):
        raise ValueError(f'the path={path} does not exist.')

    ament_dependencies_str = ""
    if vars(args)['ament_dependencies'] is not None:
        for ament_dependency in args.ament_dependencies:
            ament_dependencies_str = ament_dependencies_str + f'find_package({ament_dependency} REQUIRED)\n'
        ament_dependencies_str = ament_dependencies_str + '  '

    cmakelists_txt_str = textwrap.dedent(f"""\
cmake_minimum_required(VERSION 3.8)
project({args.package_name})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
{ament_dependencies_str}\
{get_cmake_for_library(path, args)}\
{get_cmake_for_nodes(path, args)}
if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()\
    """)

    with open(path / pathlib.Path('CMakeLists.txt'), 'w+') as cmakelists_txt_file:
        cmakelists_txt_file.write(cmakelists_txt_str)
