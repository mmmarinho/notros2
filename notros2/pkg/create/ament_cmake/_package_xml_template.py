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


def create_package_xml_from_template(path: pathlib.Path, config: dict, args: argparse.Namespace):
    """
    Creates the package.xml file for the package.
    :param path: the root directory of the package.
    :param config: the config file.
    :param args: the parsed commandline arguments.
    :return: None
    :except: Raises ValueErrors when the inputs are wrong.
    """
    _check_common_inputs(path, args)

    print(f"Creating package.xml... ")

    ament_dependencies_str = ""
    if args.ament_dependencies is not None:
        ament_dependencies_str = ament_dependencies_str + '  \n'
        for ament_dependency in args.ament_dependencies:
            ament_dependencies_str = ament_dependencies_str + f'          <dependency>{ament_dependency}</dependency>\n'
        ament_dependencies_str = ament_dependencies_str + '  '

    package_xml_str = textwrap.dedent(f"""\
        <?xml version="1.0"?>
        <?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
        <package format="3">
          <name>{args.package_name}</name>
          <version>0.0.0</version>
          <description>TODO: Package description</description>
          <maintainer email="{config['mantainer_email']}">{config['mantainer_name']}</maintainer>
          <license>{config['license']}</license>
        
          <buildtool_depend>{args.build_type}</buildtool_depend>
          {ament_dependencies_str:>8}
          <test_depend>ament_lint_auto</test_depend>
          <test_depend>ament_lint_common</test_depend>
        
          <export>
            <build_type>{args.build_type}</build_type>
          </export>
        </package>\
    """
                                      )

    with open(path / pathlib.Path('package.xml'), 'w+') as package_xml_file:
        package_xml_file.write(package_xml_str)
