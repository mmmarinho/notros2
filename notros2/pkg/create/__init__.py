import os
import argparse
import pathlib

from .ament_cmake import \
    create_package_xml_from_template, \
    create_cmakelists_txt_from_template, \
    create_cpp_for_nodes, \
    create_hpp_for_nodes


def create_package_template(path: pathlib.Path, config: dict, args: argparse.Namespace) -> None:
    package_path = path/pathlib.Path(args.package_name)
    if not os.path.isdir(package_path):
        os.mkdir(package_path)
    else:
        raise ValueError(f"The directory {args.package_name} already exists, choose another name for your package.")

    create_package_xml_from_template(package_path, config, args)
    create_cmakelists_txt_from_template(package_path, args)
    create_cpp_for_nodes(package_path, args)
