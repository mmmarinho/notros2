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

from notros2.pkg.create._commons import _check_common_inputs


def create_module(path: pathlib.Path, args: argparse.Namespace) -> None:
    """
    Creates the resource folder/file or an ament_python package.
    :param path: the path to the root folder of the package.
    :param args: the parsed commandline arguments.
    :return: None
    :except: Raises ValueErrors when the inputs are wrong.
    """
    _check_common_inputs(path, args)

    print(f"Creating Python module ...")

    module_folder_path = path / pathlib.Path(args.package_name)
    if not os.path.isdir(module_folder_path):
        os.mkdir(module_folder_path)

    with open(module_folder_path / pathlib.Path("__init__.py"), 'w+') as init_file:
        init_file.write("")
