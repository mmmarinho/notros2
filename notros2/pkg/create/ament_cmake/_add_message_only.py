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
from ._commons import \
    _check_common_inputs, \
    _parse_ament_dependencies_for_cmake
from notros2.pkg.create.ament_cmake.templates.interfaces_only import cmake, msg, srv


def get_cmake_for_interfaces_only(path: pathlib.Path, args: argparse.Namespace):
    _check_common_inputs(path, args)

    if not args.interfaces_only:
        return ""

    print(f"Adding CMakeLists.txt directive for interfaces only package ...")

    ament_dependencies_str = _parse_ament_dependencies_for_cmake(args)

    context = {'ament_dependencies_str': ament_dependencies_str}
    add_interfaces_only_str = cmake.get_source(args, context)

    return add_interfaces_only_str


def create_msg_for_interfaces_only(path: pathlib.Path, args: argparse.Namespace) -> None:
    _check_common_inputs(path, args)

    if not args.interfaces_only:
        return

    msg_source_path = path / pathlib.Path('msg')
    if not os.path.isdir(msg_source_path):
        os.mkdir(msg_source_path)

    print(f"Creating sample msg ...")

    msg_str = msg.get_source(args, context={})
    with open(msg_source_path / pathlib.Path(f'AmazingQuote.msg'), 'w+') as msg_file:
        msg_file.write(msg_str)


def create_srv_for_interfaces_only(path: pathlib.Path, args: argparse.Namespace) -> None:
    _check_common_inputs(path, args)

    if not args.interfaces_only:
        return

    srv_source_path = path / pathlib.Path('srv')
    if not os.path.isdir(srv_source_path):
        os.mkdir(srv_source_path)

    print(f"Creating sample srv ...")

    srv_str = srv.get_source(args, context={})
    with open(srv_source_path / pathlib.Path(f'ReviewAQuote.srv'), 'w+') as srv_file:
        srv_file.write(srv_str)
