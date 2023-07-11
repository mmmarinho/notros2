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
from notros2.pkg.create.ament_cmake.templates.simple_node import cpp, main_cpp, hpp, cmake


def create_cpp_for_nodes(path: pathlib.Path, args: argparse.Namespace) -> None:
    _check_common_inputs(path, args)

    if vars(args)['add_nodes'] is None:
        return
    cpp_source_path = path / pathlib.Path('src')
    if not os.path.isdir(cpp_source_path):
        os.mkdir(cpp_source_path)

    for node_name in vars(args)['add_nodes']:
        print(f"Creating {node_name}.cpp ...")
        NodeName = node_name.replace("_", " ").title().replace(" ", "")

        node_cpp_str = cpp.get_source(node_name, NodeName)

        with open(cpp_source_path / pathlib.Path(f'{node_name}.cpp'), 'w+') as node_cpp_file:
            node_cpp_file.write(node_cpp_str)

        print(f"Creating {node_name}_main.cpp ...")

        context = {
            'node_name': node_name,
            'NodeName': NodeName
        }
        node_cpp_main_str = main_cpp.get_source(args, context)

        with open(cpp_source_path / pathlib.Path(f'{node_name}_main.cpp'), 'w+') as node_cpp_main_file:
            node_cpp_main_file.write(node_cpp_main_str)


def create_hpp_for_nodes(path: pathlib.Path, args: argparse.Namespace) -> None:
    _check_common_inputs(path, args)

    if vars(args)['add_nodes'] is None:
        return
    cpp_source_path = path / pathlib.Path('src')
    if not os.path.isdir(cpp_source_path):
        os.mkdir(cpp_source_path)

    for node_name in vars(args)['add_nodes']:
        print(f"Creating {node_name}.hpp ...")

        NodeName = node_name.replace("_", " ").title().replace(" ", "")

        context = {
            "NodeName": NodeName
        }
        node_hpp_str = hpp.get_source(args, context)

        with open(cpp_source_path / pathlib.Path(f'{node_name}.hpp'), 'w+') as node_hpp_file:
            node_hpp_file.write(node_hpp_str)


def get_cmake_for_nodes(path: pathlib.Path, args: argparse.Namespace) -> str:
    _check_common_inputs(path, args)

    if vars(args)['add_nodes'] is None:
        return ""

    ament_dependencies_str = ""
    if vars(args)['ament_dependencies'] is not None:
        for ament_dependency in args.ament_dependencies:
            ament_dependencies_str = ament_dependencies_str + f'    {ament_dependency}\n'

    add_nodes_str = ""
    for node_name in vars(args)['add_nodes']:
        print(f"Adding CMakeLists.txt directive for {node_name} ...")

        nl = '\n'

        context = {
            "node_name": node_name,
            "ament_dependencies_str": ament_dependencies_str,
            "nl": nl
        }
        add_node_str = cmake.get_source(args, context)

        add_nodes_str = add_nodes_str + add_node_str

    return add_nodes_str
