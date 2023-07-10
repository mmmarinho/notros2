import os
import textwrap
import argparse
import pathlib


def get_cmake_for_library(path: pathlib.Path, args: argparse.Namespace):
    if path is None:
        raise ValueError('Path must not be None.')
    if args is None:
        raise ValueError('args must not be None.')
    if not os.path.isdir(path):
        raise ValueError(f'the path={path} does not exist.')
    if not args.has_library:
        return ""

    ament_dependencies_str = ""
    if args.ament_dependencies is not None:
        for ament_dependency in args.ament_dependencies:
            ament_dependencies_str = ament_dependencies_str + f'        {ament_dependency}\n'

    add_lib_str = textwrap.dedent(f"""
    ####################################
    # CPP Shared Library Block [BEGIN] #
    # vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv #
    # https://ros2-tutorial.readthedocs.io/en/latest/
    # The most common use case is to merge everything you need to export
    # into the same shared library called ${{PROJECT_NAME}}.
    add_library(${{PROJECT_NAME}} SHARED
        src/sample_class.cpp
    
        )
    
    ament_target_dependencies(${{PROJECT_NAME}}
    {ament_dependencies_str}\
    
        )
    
    target_include_directories(${{PROJECT_NAME}}
        PUBLIC
        $<BUILD_INTERFACE:${{CMAKE_CURRENT_SOURCE_DIR}}/include>
        $<INSTALL_INTERFACE:include>)
    
    ament_export_targets(export_${{PROJECT_NAME}} HAS_LIBRARY_TARGET)
    ament_export_dependencies(
        # ament_dependencies, e.g. other ros2 packages
    {ament_dependencies_str}\
    
        # cmake and other system dependencies
        Eigen3
        Qt5Core
    
        )
    
    target_link_libraries(${{PROJECT_NAME}}
        Qt5::Core
    
        )
    
    install(
        DIRECTORY include/
        DESTINATION include
        )
    
    install(
        TARGETS ${{PROJECT_NAME}}
        EXPORT export_${{PROJECT_NAME}}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
        )
    # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ #
    # CPP Shared Library Block [END] #
    ##################################
    """)

    return add_lib_str





