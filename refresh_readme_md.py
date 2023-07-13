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

        return """
### {}

```commandline
{}
```

that outputs

```commandline
{}\
```
""".format(
            self._description,
            " ".join(self._command),
            sp.stdout + sp.stderr
        )


# First line is the description, the following line is the command
command_str = """\
Define default values
notros2 pkg set -h
Define default mantainer name
notros2 pkg set --mantainer-name "John"
Define default mantainer email
notros2 pkg set --mantainer-email "john.testerson@testcorp.info"
Define default license
notros2 pkg set --license "Licenest"
Create ROS2 packages
notros2 pkg create -h
Create empty ament_python package
notros2 pkg create ap_test_package1 ament_python
Create ament_python package with dependencies
notros2 pkg create ap_test_package2 ament_python --ament-dependencies rclcpp geometry_msgs
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
# (Murilo's) notros2 Utilities 

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
