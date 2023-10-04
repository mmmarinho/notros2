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

from notros2.pkg.create._commons import _check_common_inputs


def create_setup_cfg(path: pathlib.Path, args: argparse.Namespace) -> None:
    """
    Creates the setup.cfg or an ament_python package.
    :param path: the path to the root folder of the package.
    :param args: the parsed commandline arguments.
    :return: None
    :except: Raises ValueErrors when the inputs are wrong.
    """
    _check_common_inputs(path, args)

    print(f"Creating setup.cfg ... ")

    setup_cfg_str = textwrap.dedent(f"""\
[develop]
script_dir=$base/lib/{args.package_name}
[install]
install_scripts=$base/lib/{args.package_name}
\
    """)

    with open(path / pathlib.Path('setup.cfg'), 'w+') as setup_cfg_file:
        setup_cfg_file.write(setup_cfg_str)
