import argparse
import sys
import pathlib
from _configuration_file_manager import make_or_get_default_user_configuration
from pkg.create import create_package_template


def main() -> None:
    try:

        parser = argparse.ArgumentParser(prog='notros2',
                                         description='notROS2 utilities.',
                                         epilog='TODO',
                                         )
        subparsers = parser.add_subparsers(help='sub-command help', required=True)

        # Current configuration
        config = make_or_get_default_user_configuration()

        # "set"
        parser_set = subparsers.add_parser('set', help='get notros2 default info.')
        set_arguments: list[str] = ['mantainer-name', 'mantainer-email', 'license']

        for argument in set_arguments:
            parser_set.add_argument(f'--{argument}',
                                    action='store',
                                    type=str,
                                    help=f"sets the default {argument}. Currently '{config['mantainer_name']}'"
                                    )

        # "pkg"
        parser_pkg = subparsers.add_parser('pkg', help='utilities for notros2 packages.')
        subparsers_pkg = parser_pkg.add_subparsers(help='sub-command help', required=True)

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
                                       help="The name for a sample library.",
                                       action="store_true",
                                       default=False
                                       )

        # print(sys.argv)
        args = parser.parse_args(sys.argv[1:])

        create_package_template(pathlib.Path("."), config, args)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print("Error ", e)


if __name__ == "__main__":
    main()
