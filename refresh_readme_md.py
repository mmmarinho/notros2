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
# Don't forget to 'brew install tree' if on macos
import subprocess
import io


class CommandPrinter:
    def __init__(self,
                 command: list[str] = None,
                 description: str = None
                 ):
        self._command = command
        self._description = description

    def __repr__(self) -> str:
        sp = subprocess.run(
            self._command,
            capture_output=True,
            text=True)

        follow_up: str = ""
        if self._description[:6] == "Create":
            fu = subprocess.run(
                ["tree", self._command[3]],
                capture_output=True,
                text=True)
            follow_up = """
           
and a folder structure

```commandline
{}\
```
            
""".format(fu.stdout + fu.stderr)

        return """
#### {}

```commandline
{}
```

that outputs

```commandline
{}\
```{}
""".format(
            self._description,
            " ".join(self._command),
            sp.stdout + sp.stderr,
            follow_up
        )


# First line is the description, the following line is the command
command_str = """\
Get general help, when you have no idea what is going on
notros2 -h
Command to define default values for new ROS2 packages
notros2 pkg set -h
Define default mantainer name
notros2 pkg set --mantainer-name "John"
Define default mantainer email
notros2 pkg set --mantainer-email "john.testerson@testcorp.info"
Define default license
notros2 pkg set --license "Licenest"
Command to create new ROS2 packages
notros2 pkg create -h
Create empty ament_python package
notros2 pkg create ap_test_package1 ament_python
Create ament_python package with dependencies
notros2 pkg create ap_test_package2 ament_python --ament-dependencies rclpy geometry_msgs
Create ament_python package with a sample library
notros2 pkg create ap_test_package3 ament_python --ament-dependencies rclpy geometry_msgs --has-library
Create ament_python package with sample nodes
notros2 pkg create ap_test_package4 ament_python --ament-dependencies rclpy geometry_msgs --add-nodes node1 node2
Create ament_python with sample nodes and a library
notros2 pkg create ap_test_package5 ament_python \
--ament-dependencies rclpy geometry_msgs \
--add-nodes node1 node2 node3 \
--has-library
Create empty ament_cmake package
notros2 pkg create ac_test_package1 ament_cmake
Create ament_cmake package with dependencies
notros2 pkg create ac_test_package2 ament_cmake --ament-dependencies rclcpp geometry_msgs
Create ament_cmake package with a sample library
notros2 pkg create ac_test_package3 ament_cmake --ament-dependencies rclcpp geometry_msgs --has-library
Create ament_cmake package with sample nodes
notros2 pkg create ac_test_package4 ament_cmake --ament-dependencies rclcpp geometry_msgs --add-nodes node1 node2
Create ament_cmake with sample nodes and a library
notros2 pkg create ac_test_package5 ament_cmake \
--ament-dependencies rclcpp geometry_msgs \
--add-nodes node1 node2 node3 \
--has-library
Create a dedicated package for interfaces with dependencies
notros2 pkg create ac_test_package6 interfaces_only --ament-dependencies geometry_msgs
"""

preamble: str = """\
### `notros2` Utilities 

ROS2 utilities.

Install with

```commandline
python -m pip install notros2
```

This creates a command line entry point called `notros2`.
"""

with open("README.md", 'w') as readme_file:
    readme_file.write(preamble)

    buffer = io.StringIO(command_str)
    while True:
        description: str = buffer.readline()
        if description == "":
            break

        # Remove the "\n" with readline()[:-1]
        command: list[str] = buffer.readline()[:-1].split(" ")

        command_printer = CommandPrinter(command=command,
                                         description=description)
        readme_file.write(str(command_printer))
