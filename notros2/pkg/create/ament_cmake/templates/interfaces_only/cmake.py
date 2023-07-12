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


def get_source(args: argparse.Namespace, context: dict) -> str:
    ament_dependencies_str: str = context['ament_dependencies_str']
    return f"""
#########################################
# Interfaces Only Package Block [BEGIN] #
# vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv #
# https://ros2-tutorial.readthedocs.io/en/latest/

find_package(rosidl_default_generators REQUIRED)

#### ROS2 Interface Directives ####
set(interface_files
    # Messages
    "msg/AmazingQuote.msg"

    # Services
    "srv/ReviewAQuote.srv"

)

rosidl_generate_interfaces(${{PROJECT_NAME}}
    ${{interface_files}}
    DEPENDENCIES
{ament_dependencies_str}\

)

ament_export_dependencies(rosidl_default_runtime)

# ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ #
# Interfaces Only Package Block [END] #
#######################################\
"""
