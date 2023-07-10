import argparse


def get_source(args: argparse.Namespace, node_name: str, ament_dependencies_str: str, nl: str) -> str:
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
