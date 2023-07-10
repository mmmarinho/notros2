import os
import textwrap
import argparse
import pathlib


def create_package_xml_from_template(path: pathlib.Path, config: dict, args: argparse.Namespace):
    if path is None:
        raise ValueError('Path must not be None.')
    if args is None:
        raise ValueError('args must not be None.')
    if not os.path.isdir(path):
        raise ValueError(f'the path={path} does not exist.')

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

    with open(path/pathlib.Path('package.xml'), 'w+') as package_xml_file:
        package_xml_file.write(package_xml_str)
