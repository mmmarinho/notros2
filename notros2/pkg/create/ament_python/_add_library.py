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
from notros2.pkg.create.ament_python.templates.library import sources_py


def create_py_for_library(path: pathlib.Path, args: argparse.Namespace) -> None:
    _check_common_inputs(path, args)

    if not vars(args)['has_library']:
        return

    python_module_path = path / pathlib.Path(args.package_name)
    python_library_path = python_module_path / pathlib.Path('sample_python_library')

    if not os.path.isdir(python_library_path):
        os.mkdir(python_library_path)

    print("Creating __init__.py ...")
    with open(python_library_path / pathlib.Path('__init__.py'), 'w+') as f:
        f.write(sources_py.get_module_init(args, context={}))

    print("Creating _sample_class.py ...")
    with open(python_library_path / pathlib.Path('_sample_class.py'), 'w+') as f:
        f.write(sources_py.get_sample_class(args, context={}))

    print("Creating _sample_function.py ...")
    with open(python_library_path / pathlib.Path('_sample_function.py'), 'w+') as f:
        f.write(sources_py.get_sample_function(args, context={}))
