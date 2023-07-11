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
    return f"""\
//Template from https://ros2-tutorial.readthedocs.io/en/latest/
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
"""