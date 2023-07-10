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
import sys
import pathlib

from notros2._configuration_file_manager import make_or_get_default_user_configuration
from notros2.pkg.create import create_package_template
from notros2._configuration_file_manager import save_user_configuration


def main() -> None:
    try:

        parser = argparse.ArgumentParser(prog='notros2',
                                         description='notros2 are not ros2 utilities.',
                                         epilog='This is a package template generator based on the tutorials at'
                                                'https://ros2-tutorial.readthedocs.io/en/latest/',
                                         )
        subparsers = parser.add_subparsers(help='sub-command help', required=True, dest='command')

        # Get current configuration
        config = make_or_get_default_user_configuration()

        # "pkg"
        parser_pkg = subparsers.add_parser('pkg', help='utilities for notros2 packages.')
        subparsers_pkg = parser_pkg.add_subparsers(help='sub-command help', required=True, dest='subcommand')

        # "pkg set"
        parser_set = subparsers_pkg.add_parser('set', help='set notros2 default package template info.')
        set_arguments: list[str] = ['mantainer-name', 'mantainer-email', 'license']

        for argument in set_arguments:
            parser_set.add_argument(f'--{argument}',
                                    action='store',
                                    type=str,
                                    help=f"sets the default {argument}. Currently '{config['mantainer_name']}'"
                                    )

        # "pkg create"
        parser_pkg_create = subparsers_pkg.add_parser('create', help='create notros2 package templates.')
        parser_pkg_create.add_argument('package_name', type=str)
        parser_pkg_create.add_argument('build_type', type=str, choices=['ament_cmake', 'ament_python'])

        parser_pkg_create.add_argument('--ament-dependencies',
                                       type=str,
                                       help="The ament dependencies, e.g. ROS2 packages such as `rclcpp`.",
                                       action='store',
                                       nargs='+')

        parser_pkg_create.add_argument('--add-nodes',
                                       type=str,
                                       help="A sequence of names of sample nodes, e.g. `my_sample_node`.",
                                       action='store',
                                       nargs='+')

        parser_pkg_create.add_argument('--has-library',
                                       help="add this flag if the package should contain a library.",
                                       action="store_true",
                                       default=False
                                       )

        if len(sys.argv) > 1:
            args = parser.parse_args(sys.argv[1:])
        else:
            parser.parse_args(["-h"])
            return

        if args.command == "pkg":
            if args.subcommand == "create":
                create_package_template(pathlib.Path("."), config, args)
            elif args.subcommand == "set":
                if len(sys.argv) == 3:
                    parser.parse_args(["pkg", "set", "-h"])
                    return
                arg_vars = vars(args)
                config_changed = False
                for argument in set_arguments:
                    argument_for_dict = argument.replace("-", "_")
                    if argument_for_dict in arg_vars:
                        if arg_vars[argument_for_dict] is not None:
                            config[argument_for_dict] = arg_vars[argument_for_dict]
                            config_changed = True
                if config_changed:
                    print("Saving new user configuration with values:")
                    print(config)
                    save_user_configuration(config)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print("Error ", e)


if __name__ == "__main__":
    main()
