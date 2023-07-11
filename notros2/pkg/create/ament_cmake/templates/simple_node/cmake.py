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
    node_name: str = context['node_name']
    ament_dependencies_str: str = context['ament_dependencies_str']
    nl: str = context['nl']

    return f"""
############################
# CPP Binary Block [BEGIN] #
# vvvvvvvvvvvvvvvvvvvvvvvv #
# https://ros2-tutorial.readthedocs.io/en/latest/
# While we cant use blocks https://cmake.org/cmake/help/latest/command/block.html#command:block
# we use set--unset
set(RCLCPP_LOCAL_BINARY_NAME {node_name})

add_executable(${{RCLCPP_LOCAL_BINARY_NAME}}
    src/{node_name}_main.cpp
    src/{node_name}.cpp

    )

ament_target_dependencies(${{RCLCPP_LOCAL_BINARY_NAME}}
{ament_dependencies_str}\

    )

target_link_libraries(${{RCLCPP_LOCAL_BINARY_NAME}}
{'' if not args.has_library else '    ${PROJECT_NAME}' + nl}\

    )

target_include_directories(${{RCLCPP_LOCAL_BINARY_NAME}} PUBLIC
    $<BUILD_INTERFACE:${{CMAKE_CURRENT_SOURCE_DIR}}/include>
    $<INSTALL_INTERFACE:include>)

target_compile_features(${{RCLCPP_LOCAL_BINARY_NAME}} PUBLIC c_std_99 cxx_std_17)

install(TARGETS ${{RCLCPP_LOCAL_BINARY_NAME}}
    DESTINATION lib/${{PROJECT_NAME}})

unset(RCLCPP_LOCAL_BINARY_NAME)
# ^^^^^^^^^^^^^^^^^^^^^^ #
# CPP Binary Block [END] #
##########################
"""
