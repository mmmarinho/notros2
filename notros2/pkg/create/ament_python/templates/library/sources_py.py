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


def get_module_init(args: argparse.Namespace, context: dict) -> str:
    return f"""\
from python_package_with_a_library.sample_python_library._sample_class import SampleClass
from python_package_with_a_library.sample_python_library._sample_function import sample_function_for_square_of_sum
"""


def get_sample_function(args: argparse.Namespace, context: dict) -> str:
    return f"""\
def sample_function_for_square_of_sum(a: float, b: float) -> float:
    \"\"\"Returns the square of a sum (a + b)^2 = a^2 + 2ab + b^2\"\"\"
    return a ** 2 + 2 * a * b + b ** 2
"""


def get_sample_class(args: argparse.Namespace, context: dict) -> str:
    return f"""\
class SampleClass:
    \"\"\"A sample class to check how they can be imported by other ROS2 packages.\"\"\"

    def __init__(self, name: str):
        self._name = name

    def get_name(self) -> str:
        \"\"\"
        Gets the name of this instance.
        :return: This name.
        \"\"\"
        return self._name
"""
