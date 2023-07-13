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

import notros2.pkg.create.ament_cmake as ac
import notros2.pkg.create.ament_python as ap
from ._package_xml_template import create_package_xml_from_template


def create_package_template(path: pathlib.Path, config: dict, args: argparse.Namespace) -> None:
    """
    Creates the package template based on the input arguments and the configuration file.
    :param path: the path for where the package should be made.
    :param config: the configuration dictionary.
    :param args: the parsed commandline arguments.
    :return: None
    :except: raises ValueErrors when an input is invalid.
    """
    package_path = path / pathlib.Path(args.package_name)
    if not os.path.isdir(package_path):
        os.mkdir(package_path)
    else:
        raise ValueError(f"The directory {args.package_name} already exists, choose another name for your package.")

    if args.build_type == "ament_cmake":
        if args.interfaces_only:
            print(f"Creating interfaces_only (ament_cmake) package {args.package_name} ... ")
            create_package_xml_from_template(package_path, config, args)
            ac.create_cmakelists_txt_from_template(package_path, args)
            ac.create_msg_for_interfaces_only(package_path, args)
            ac.create_srv_for_interfaces_only(package_path, args)

            if vars(args)['add_nodes'] is not None:
                print("WARNING: --add-nodes is ignored with build-type=interfaces-only.")
            if vars(args)['has_library']:
                print("WARNING: --has-library is ignored with build-type=interfaces-only.")
        else:
            print(f"Creating ament_cmake package {args.package_name} ... ")
            create_package_xml_from_template(package_path, config, args)
            ac.create_cmakelists_txt_from_template(package_path, args)
            ac.create_cpp_for_nodes(package_path, args)
            ac.create_hpp_for_nodes(package_path, args)
            ac.create_cpp_for_library(package_path, args)
            ac.create_hpp_for_library(package_path, args)
    elif args.build_type == "ament_python":
        print(f"Creating ament_python package {args.package_name} ... ")
        create_package_xml_from_template(package_path, config, args)
        ap.create_setup_py(package_path, config, args)
        ap.create_setup_cfg(package_path, args)
        ap.create_resource_file(package_path, args)
        ap.create_module(package_path, args)
    else:
        raise NotImplementedError("How did you get here?")
