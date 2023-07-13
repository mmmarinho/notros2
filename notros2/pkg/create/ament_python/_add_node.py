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
import argparse
import pathlib
from notros2.pkg.create._commons import \
    _check_common_inputs

from notros2.pkg.create.ament_python.templates.simple_node import main_py


def get_entry_point_for_node(path: pathlib.Path, args: argparse.Namespace) -> str:
    _check_common_inputs(path, args)

    if vars(args)['add_nodes'] is None:
        return ""

    add_nodes_str = ""
    for node_name in vars(args)['add_nodes']:
        print(f"Adding entry_point directive for {node_name} ...")
        add_nodes_str = add_nodes_str + f"        '{node_name} = {args.package_name}.{node_name}:main',\n"

    # Remove the last ",\n"
    return add_nodes_str[:-2]


def create_py_for_nodes(path: pathlib.Path, args: argparse.Namespace) -> None:
    _check_common_inputs(path, args)

    if vars(args)['add_nodes'] is None:
        return

    node_source_path = path / pathlib.Path(args.package_name)

    for node_name in vars(args)['add_nodes']:
        print(f"Creating {node_name}.py ...")
        NodeName = node_name.replace("_", " ").title().replace(" ", "")

        context = {
            'node_name': node_name,
            'NodeName': NodeName
        }
        node_python_srt = main_py.get_source(args, context)

        with open(node_source_path / pathlib.Path(f'{node_name}.py'), 'w+') as node_source_file:
            node_source_file.write(node_python_srt)
