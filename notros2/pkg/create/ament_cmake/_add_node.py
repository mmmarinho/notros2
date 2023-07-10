import os
import textwrap
import argparse
import pathlib


def create_cpp_for_nodes(path: pathlib.Path, args: argparse.Namespace) -> None:
    if path is None:
        raise ValueError('Path must not be None.')
    if args is None:
        raise ValueError('args must not be None.')
    if not os.path.isdir(path):
        raise ValueError(f'the path={path} does not exist.')
    if vars(args)['add_nodes'] is None:
        return
    cpp_source_path = path / pathlib.Path('src')
    if not os.path.isdir(cpp_source_path):
        os.mkdir(cpp_source_path)

    for node_name in vars(args)['add_nodes']:
        NodeName = node_name.replace("_", " ").title().replace(" ", "")
        node_cpp_str = textwrap.dedent(f"""\
#include "{node_name}.hpp"

/**
 * @brief {NodeName}::{NodeName} Default constructor.
 */
{NodeName}::{NodeName}():
    rclcpp::Node("{node_name}"),
    timer_period_(0.5),
    print_count_(0)
{{
    timer_ = create_wall_timer(
                std::chrono::milliseconds(long(timer_period_*1e3)),
                std::bind(&{NodeName}::_timer_callback, this)
                );
}}

/**
 * @brief {NodeName}::_timer_callback periodically prints class info using RCLCPP_INFO.
 */
void {NodeName}::_timer_callback()
{{
    RCLCPP_INFO_STREAM(get_logger(),
                       std::string("Printed ") +
                       std::to_string(print_count_) +
                       std::string(" times.")
                       );
    print_count_++;
}}
""")
        with open(cpp_source_path / pathlib.Path(f'{node_name}.cpp'), 'w+') as node_cpp_file:
            node_cpp_file.write(node_cpp_str)


def create_hpp_for_nodes(path: pathlib.Path, args: argparse.Namespace) -> None:
    if path is None:
        raise ValueError('Path must not be None.')
    if args is None:
        raise ValueError('args must not be None.')
    if not os.path.isdir(path):
        raise ValueError(f'the path={path} does not exist.')
    if vars(args)['add_nodes'] is None:
        return
    cpp_source_path = path / pathlib.Path('src')
    if not os.path.isdir(cpp_source_path):
        os.mkdir(cpp_source_path)

    for node_name in vars(args)['add_nodes']:
        NodeName = node_name.replace("_", " ").title().replace(" ", "")
        node_hpp_str = textwrap.dedent(f"""\
#pragma once

#include <rclcpp/rclcpp.hpp>

/**
 * @brief A ROS2 Node that prints to the console periodically, but in C++.
 */
class {NodeName}: public rclcpp::Node
{{
private:
    double timer_period_;
    int print_count_;
    rclcpp::TimerBase::SharedPtr timer_;

    void _timer_callback();
public:
    {NodeName};

}};
""")
        with open(cpp_source_path / pathlib.Path(f'{node_name}.cpp'), 'w+') as node_hpp_file:
            node_hpp_file.write(node_hpp_str)


def get_cmake_for_nodes(path: pathlib.Path, args: argparse.Namespace) -> str:
    if path is None:
        raise ValueError('Path must not be None.')
    if args is None:
        raise ValueError('args must not be None.')
    if not os.path.isdir(path):
        raise ValueError(f'the path={path} does not exist.')
    if vars(args)['add_nodes'] is None:
        return ""

    ament_dependencies_str = ""
    if vars(args)['ament_dependencies'] is not None:
        for ament_dependency in args.ament_dependencies:
            ament_dependencies_str = ament_dependencies_str + f'    {ament_dependency}\n'

    add_nodes_str = ""
    for node_name in vars(args)['add_nodes']:
        nl = '\n'
        add_node_str = textwrap.dedent(f"""
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
    """)
        add_nodes_str = add_nodes_str + add_node_str

    return add_nodes_str
