"""
MIT LICENSE

Copyright (C) 2023 Murilo Marques Marinho (www.murilomarinho.info)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""
import textwrap
import argparse
import pathlib

from ._input_checker import _check_common_inputs
from ._add_node import get_cmake_for_nodes
from ._add_library import get_cmake_for_library


def create_cmakelists_txt_from_template(path: pathlib.Path, args: argparse.Namespace) -> None:
    """
    Creates the CMakeLists.txt for an ament_cmake package.
    :param path: the path to the root folder of the package.
    :param args: the parsed commandline arguments.
    :return: None
    :except: Raises ValueErrors when the inputs are wrong.
    """
    _check_common_inputs(path, args)

    print(f"Creating CMakeLists.txt... ")

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
