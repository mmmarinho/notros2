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
import os
import argparse
import pathlib
from ._input_checker import _check_common_inputs
from notros2.pkg.create.ament_cmake.templates.library import cpp, hpp, cmake


def create_hpp_for_library(path: pathlib.Path, args: argparse.Namespace) -> None:
    _check_common_inputs(path, args)

    include_path = path / pathlib.Path('include')
    package_include_path = include_path / pathlib.Path(f'{args.package_name}')
    if not os.path.isdir(include_path):
        os.mkdir(include_path)
        if not os.path.isdir(package_include_path):
            os.mkdir(package_include_path)

    if vars(args)['has_library'] is False:
        print(f"Creating sample placeholder for library hpp ...")
        with open(package_include_path / pathlib.Path('.placeholder'), 'w+') as _:
            pass
    else:
        print(f"Creating sample library hpp ...")
        library_hpp_str = hpp.get_source(args, context={})
        with open(package_include_path / pathlib.Path('sample_class.hpp'), 'w+') as library_hpp_file:
            library_hpp_file.write(library_hpp_str)


def create_cpp_for_library(path: pathlib.Path, args: argparse.Namespace) -> None:
    _check_common_inputs(path, args)

    if vars(args)['has_library'] is False:
        return
    library_source_path = path / pathlib.Path('src')
    if not os.path.isdir(library_source_path):
        os.mkdir(library_source_path)

    print(f"Creating sample library cpp ...")

    library_cpp_str = cpp.get_source(args, context={})
    with open(library_source_path / pathlib.Path(f'sample_class.cpp'), 'w+') as library_cpp_file:
        library_cpp_file.write(library_cpp_str)


def get_cmake_for_library(path: pathlib.Path, args: argparse.Namespace):
    _check_common_inputs(path, args)

    if not args.has_library:
        return ""

    print(f"Adding CMakeLists.txt directive for library ...")

    ament_dependencies_str = ""
    if args.ament_dependencies is not None:
        for ament_dependency in args.ament_dependencies:
            ament_dependencies_str = ament_dependencies_str + f'    {ament_dependency}\n'

    context = {'ament_dependencies_str': ament_dependencies_str}
    add_lib_str = cmake.get_source(args, context)

    return add_lib_str
