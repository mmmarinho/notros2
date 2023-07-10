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
import os
import textwrap
import argparse
import pathlib


def create_hpp_for_library(path: pathlib.Path, args: argparse.Namespace) -> None:
    if path is None:
        raise ValueError('Path must not be None.')
    if args is None:
        raise ValueError('args must not be None.')
    if not os.path.isdir(path):
        raise ValueError(f'the path={path} does not exist.')
    if vars(args)['has_library'] is False:
        return

    include_path = path / pathlib.Path('include')
    package_include_path = include_path / pathlib.Path(f'{args.package_name}')
    if not os.path.isdir(include_path):
        os.mkdir(include_path)
        if not os.path.isdir(package_include_path):
            os.mkdir(package_include_path)

    library_hpp_str = textwrap.dedent(f"""\
#pragma once

#include <ostream>

#include <QString>
#include <eigen3/Eigen/Dense>

class SampleClass
{{
private:
    int a_private_member_;
    const QString a_private_qt_string_;
    const Eigen::MatrixXd a_private_eigen3_matrix_;

public:
    SampleClass();

    int get_a_private_member() const;
    void set_a_private_member(int value);
    std::string to_string() const;

    static double sum_of_squares(const double&a, const double& b);
}};

std::ostream& operator<<(std::ostream& os, const SampleClass& sc);
""")
    with open(package_include_path / pathlib.Path(f'sample_class.hpp'), 'w+') as library_hpp_file:
        library_hpp_file.write(library_hpp_str)


def create_cpp_for_library(path: pathlib.Path, args: argparse.Namespace) -> None:
    if path is None:
        raise ValueError('Path must not be None.')
    if args is None:
        raise ValueError('args must not be None.')
    if not os.path.isdir(path):
        raise ValueError(f'the path={path} does not exist.')
    if vars(args)['has_library'] is False:
        return
    library_source_path = path / pathlib.Path('src')
    if not os.path.isdir(library_source_path):
        os.mkdir(library_source_path)

    library_cpp_str = textwrap.dedent(f"""\
#include <{args.package_name}/sample_class.hpp>



/**
 * @brief SampleClass::SampleClass the default constructor.
 */
SampleClass::SampleClass():
    a_private_qt_string_("I am a QString"),
    a_private_eigen3_matrix_((Eigen::Matrix2d() << 1,2,3,4).finished())
{{

}}

/**
 * @brief SampleClass::get_a_private_member.
 * @return an int with the value of a_private_member_.
 */
int SampleClass::get_a_private_member() const
{{
    return a_private_member_;
}}

/**
 * @brief SampleClass::set_a_private_member.
 * @param [in] value The new value for a_private_member_.
 */
void SampleClass::set_a_private_member(int value)
{{
    a_private_member_ = value;
}}

/**
 * @brief SampleClass::sum_of_squares.
 * @param [in] a The first number.
 * @param [in] b The second number.
 * @return a*a + 2*a*b + b*b.
 */
double SampleClass::sum_of_squares(const double &a, const double &b)
{{
    return a*a + 2*a*b + b*b;
}}

/**
 * @brief SampleClass::to_string converts a SampleClass to a std::string representation.
 * @return a pretty(-ish) std::string representation of the object.
 */
std::string SampleClass::to_string() const
{{
    std::stringstream ss;
    ss << "Sample_Class:: " << std::endl <<
          "a_private_member_ = "        << std::to_string(a_private_member_) << std::endl <<
          "a_private_qt_string_ = "     << a_private_qt_string_.toStdString() << std::endl <<
          "a_private_eigen3_matrix_ = " << a_private_eigen3_matrix_ << std::endl;
    return ss.str();
}}

/**
 * @brief operator << the stream operator for SampleClass objects.
 * @param [in/out] the std::ostream to be modified.
 * @param [in] sc the SampleClass whose representation is to be streamed.
 * @return the modified os with the added SampleClass string representation.
 * @see SampleClass::to_string().
 */
std::ostream &operator<<(std::ostream &os, const SampleClass &sc)
{{
    return os << sc.to_string();
}}
""")
    with open(library_source_path / pathlib.Path(f'sample_class.cpp'), 'w+') as library_cpp_file:
        library_cpp_file.write(library_cpp_str)


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
