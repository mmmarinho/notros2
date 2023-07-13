# (Murilo's) not ROS Utilities 

ROS2 utilities.

Install with

```commandline
python -m pip install notros2
```

This creates a command line entry point called `notros2`.

### Define default values


```commandline
notros2 pkg set -h
```

that outputs

```commandline
usage: notros2 pkg set [-h] [--mantainer-name MANTAINER_NAME]
                       [--mantainer-email MANTAINER_EMAIL] [--license LICENSE]

optional arguments:
  -h, --help            show this help message and exit
  --mantainer-name MANTAINER_NAME
                        sets the default mantainer-name. Currently 'TODO'
  --mantainer-email MANTAINER_EMAIL
                        sets the default mantainer-email. Currently
                        'TODO@TO.DO'
  --license LICENSE     sets the default license. Currently 'TODO'
```

### Define default mantainer name


```commandline
notros2 pkg set --mantainer-name "John"
```

that outputs

```commandline
Saving new user configuration with values:
{'mantainer_name': '"John"', 'mantainer_email': 'TODO@TO.DO', 'license': 'TODO'}
```

### Define default mantainer email


```commandline
notros2 pkg set --mantainer-email "john.testerson@testcorp.info"
```

that outputs

```commandline
Saving new user configuration with values:
{'mantainer_name': '"John"', 'mantainer_email': '"john.testerson@testcorp.info"', 'license': 'TODO'}
```

### Define default license


```commandline
notros2 pkg set --license "Licenest"
```

that outputs

```commandline
Saving new user configuration with values:
{'mantainer_name': '"John"', 'mantainer_email': '"john.testerson@testcorp.info"', 'license': '"Licenest"'}
```

### Create ROS2 packages


```commandline
notros2 pkg create -h
```

that outputs

```commandline
usage: notros2 pkg create [-h]
                          [--ament-dependencies AMENT_DEPENDENCIES [AMENT_DEPENDENCIES ...]]
                          [--add-nodes ADD_NODES [ADD_NODES ...]]
                          [--has-library]
                          package_name
                          {ament_cmake,ament_python,interfaces_only}

positional arguments:
  package_name
  {ament_cmake,ament_python,interfaces_only}

optional arguments:
  -h, --help            show this help message and exit
  --ament-dependencies AMENT_DEPENDENCIES [AMENT_DEPENDENCIES ...]
                        The ament dependencies, e.g. ROS2 packages such as
                        `rclcpp`.
  --add-nodes ADD_NODES [ADD_NODES ...]
                        A sequence of names of sample nodes, e.g.
                        `my_sample_node`.
  --has-library         add this flag if the package should contain a library.
```

### Create empty ament_cmake package


```commandline
notros2 pkg create test_package1 ament_cmake
```

that outputs

```commandline
Creating ament_cmake package test_package1... 
Creating package.xml... 
Creating CMakeLists.txt... 
Creating sample placeholder for library hpp ...
```

### Create ament_cmake package with dependencies


```commandline
notros2 pkg create test_package2 ament_cmake --ament-dependencies rclcpp geometry_msgs
```

that outputs

```commandline
Creating ament_cmake package test_package2... 
Creating package.xml... 
Creating CMakeLists.txt... 
Creating sample placeholder for library hpp ...
```

### Create ament_cmake package with a sample library


```commandline
notros2 pkg create test_package3 ament_cmake --ament-dependencies rclcpp geometry_msgs --has-library
```

that outputs

```commandline
Creating ament_cmake package test_package3... 
Creating package.xml... 
Creating CMakeLists.txt... 
Adding CMakeLists.txt directive for library ...
Creating sample library cpp ...
Creating sample library hpp ...
```

### Create ament_cmake package with sample nodes


```commandline
notros2 pkg create test_package4 ament_cmake --ament-dependencies rclcpp geometry_msgs --add-nodes node1 node2
```

that outputs

```commandline
Creating ament_cmake package test_package4... 
Creating package.xml... 
Creating CMakeLists.txt... 
Adding CMakeLists.txt directive for node1 ...
Adding CMakeLists.txt directive for node2 ...
Creating node1.cpp ...
Creating node1_main.cpp ...
Creating node2.cpp ...
Creating node2_main.cpp ...
Creating node1.hpp ...
Creating node2.hpp ...
Creating sample placeholder for library hpp ...
```

### Create ament_cmake with sample nodes and a library


```commandline
notros2 pkg create test_package5 ament_cmake --ament-dependencies rclcpp geometry_msgs --add-nodes node1 node2 node3 --has-library
```

that outputs

```commandline
Creating ament_cmake package test_package5... 
Creating package.xml... 
Creating CMakeLists.txt... 
Adding CMakeLists.txt directive for library ...
Adding CMakeLists.txt directive for node1 ...
Adding CMakeLists.txt directive for node2 ...
Adding CMakeLists.txt directive for node3 ...
Creating node1.cpp ...
Creating node1_main.cpp ...
Creating node2.cpp ...
Creating node2_main.cpp ...
Creating node3.cpp ...
Creating node3_main.cpp ...
Creating node1.hpp ...
Creating node2.hpp ...
Creating node3.hpp ...
Creating sample library cpp ...
Creating sample library hpp ...
```

### Create a dedicated package for interfaces with dependencies


```commandline
notros2 pkg create test_package6 interfaces_only --ament-dependencies geometry_msgs
```

that outputs

```commandline
Creating interfaces_only (ament_cmake) package test_package6... 
Creating package.xml... 
Creating CMakeLists.txt... 
Adding CMakeLists.txt directive for interfaces only package ...
Creating sample msg ...
Creating sample srv ...
```
