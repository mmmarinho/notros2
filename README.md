### `notros2` Utilities 

ROS2 utilities.

Install with

```commandline
python -m pip install notros2
```

This creates a command line entry point called `notros2`.

#### Command to define default values for new ROS2 packages


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

#### Define default mantainer name


```commandline
notros2 pkg set --mantainer-name "John"
```

that outputs

```commandline
Saving new user configuration with values:
{'mantainer_name': '"John"', 'mantainer_email': 'TODO@TO.DO', 'license': 'TODO'}
```

#### Define default mantainer email


```commandline
notros2 pkg set --mantainer-email "john.testerson@testcorp.info"
```

that outputs

```commandline
Saving new user configuration with values:
{'mantainer_name': '"John"', 'mantainer_email': '"john.testerson@testcorp.info"', 'license': 'TODO'}
```

#### Define default license


```commandline
notros2 pkg set --license "Licenest"
```

that outputs

```commandline
Saving new user configuration with values:
{'mantainer_name': '"John"', 'mantainer_email': '"john.testerson@testcorp.info"', 'license': '"Licenest"'}
```

#### Command to create new ROS2 packages


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

#### Create empty ament_python package


```commandline
notros2 pkg create ap_test_package1 ament_python
```

that outputs

```commandline
Creating ament_python package ap_test_package1 ... 
Creating package.xml... 
Creating setup.py ... 
Creating setup.cfg ... 
Creating resource folder/file ...
Creating Python module ...
```
           
and a folder structure

```commandline
ap_test_package1
├── ap_test_package1
│   └── __init__.py
├── package.xml
├── resource
│   └── ap_test_package1
├── setup.cfg
└── setup.py

3 directories, 5 files
```
            


#### Create ament_python package with dependencies


```commandline
notros2 pkg create ap_test_package2 ament_python --ament-dependencies rclpy geometry_msgs
```

that outputs

```commandline
Creating ament_python package ap_test_package2 ... 
Creating package.xml... 
Creating setup.py ... 
Creating setup.cfg ... 
Creating resource folder/file ...
Creating Python module ...
```
           
and a folder structure

```commandline
ap_test_package2
├── ap_test_package2
│   └── __init__.py
├── package.xml
├── resource
│   └── ap_test_package2
├── setup.cfg
└── setup.py

3 directories, 5 files
```
            


#### Create ament_python package with a sample library


```commandline
notros2 pkg create ap_test_package3 ament_python --ament-dependencies rclpy geometry_msgs --has-library
```

that outputs

```commandline
Creating ament_python package ap_test_package3 ... 
Creating package.xml... 
Creating setup.py ... 
Creating setup.cfg ... 
Creating resource folder/file ...
Creating Python module ...
Creating __init__.py ...
Creating _sample_class.py ...
Creating _sample_function.py ...
```
           
and a folder structure

```commandline
ap_test_package3
├── ap_test_package3
│   ├── __init__.py
│   └── sample_python_library
│       ├── __init__.py
│       ├── _sample_class.py
│       └── _sample_function.py
├── package.xml
├── resource
│   └── ap_test_package3
├── setup.cfg
└── setup.py

4 directories, 8 files
```
            


#### Create ament_python package with sample nodes


```commandline
notros2 pkg create ap_test_package4 ament_python --ament-dependencies rclpy geometry_msgs --add-nodes node1 node2
```

that outputs

```commandline
Creating ament_python package ap_test_package4 ... 
Creating package.xml... 
Creating setup.py ... 
Adding entry_point directive for node1 ...
Adding entry_point directive for node2 ...
Creating setup.cfg ... 
Creating resource folder/file ...
Creating Python module ...
Creating node1.py ...
Creating node2.py ...
```
           
and a folder structure

```commandline
ap_test_package4
├── ap_test_package4
│   ├── __init__.py
│   ├── node1.py
│   └── node2.py
├── package.xml
├── resource
│   └── ap_test_package4
├── setup.cfg
└── setup.py

3 directories, 7 files
```
            


#### Create ament_python with sample nodes and a library


```commandline
notros2 pkg create ap_test_package5 ament_python --ament-dependencies rclpy geometry_msgs --add-nodes node1 node2 node3 --has-library
```

that outputs

```commandline
Creating ament_python package ap_test_package5 ... 
Creating package.xml... 
Creating setup.py ... 
Adding entry_point directive for node1 ...
Adding entry_point directive for node2 ...
Adding entry_point directive for node3 ...
Creating setup.cfg ... 
Creating resource folder/file ...
Creating Python module ...
Creating node1.py ...
Creating node2.py ...
Creating node3.py ...
Creating __init__.py ...
Creating _sample_class.py ...
Creating _sample_function.py ...
```
           
and a folder structure

```commandline
ap_test_package5
├── ap_test_package5
│   ├── __init__.py
│   ├── node1.py
│   ├── node2.py
│   ├── node3.py
│   └── sample_python_library
│       ├── __init__.py
│       ├── _sample_class.py
│       └── _sample_function.py
├── package.xml
├── resource
│   └── ap_test_package5
├── setup.cfg
└── setup.py

4 directories, 11 files
```
            


#### Create empty ament_cmake package


```commandline
notros2 pkg create ac_test_package1 ament_cmake
```

that outputs

```commandline
Creating ament_cmake package ac_test_package1 ... 
Creating package.xml... 
Creating CMakeLists.txt... 
Creating sample placeholder for library hpp ...
```
           
and a folder structure

```commandline
ac_test_package1
├── CMakeLists.txt
├── include
│   └── ac_test_package1
└── package.xml

3 directories, 2 files
```
            


#### Create ament_cmake package with dependencies


```commandline
notros2 pkg create ac_test_package2 ament_cmake --ament-dependencies rclcpp geometry_msgs
```

that outputs

```commandline
Creating ament_cmake package ac_test_package2 ... 
Creating package.xml... 
Creating CMakeLists.txt... 
Creating sample placeholder for library hpp ...
```
           
and a folder structure

```commandline
ac_test_package2
├── CMakeLists.txt
├── include
│   └── ac_test_package2
└── package.xml

3 directories, 2 files
```
            


#### Create ament_cmake package with a sample library


```commandline
notros2 pkg create ac_test_package3 ament_cmake --ament-dependencies rclcpp geometry_msgs --has-library
```

that outputs

```commandline
Creating ament_cmake package ac_test_package3 ... 
Creating package.xml... 
Creating CMakeLists.txt... 
Adding CMakeLists.txt directive for library ...
Creating sample library cpp ...
Creating sample library hpp ...
```
           
and a folder structure

```commandline
ac_test_package3
├── CMakeLists.txt
├── include
│   └── ac_test_package3
│       └── sample_class.hpp
├── package.xml
└── src
    └── sample_class.cpp

4 directories, 4 files
```
            


#### Create ament_cmake package with sample nodes


```commandline
notros2 pkg create ac_test_package4 ament_cmake --ament-dependencies rclcpp geometry_msgs --add-nodes node1 node2
```

that outputs

```commandline
Creating ament_cmake package ac_test_package4 ... 
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
           
and a folder structure

```commandline
ac_test_package4
├── CMakeLists.txt
├── include
│   └── ac_test_package4
├── package.xml
└── src
    ├── node1.cpp
    ├── node1.hpp
    ├── node1_main.cpp
    ├── node2.cpp
    ├── node2.hpp
    └── node2_main.cpp

4 directories, 8 files
```
            


#### Create ament_cmake with sample nodes and a library


```commandline
notros2 pkg create ac_test_package5 ament_cmake --ament-dependencies rclcpp geometry_msgs --add-nodes node1 node2 node3 --has-library
```

that outputs

```commandline
Creating ament_cmake package ac_test_package5 ... 
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
           
and a folder structure

```commandline
ac_test_package5
├── CMakeLists.txt
├── include
│   └── ac_test_package5
│       └── sample_class.hpp
├── package.xml
└── src
    ├── node1.cpp
    ├── node1.hpp
    ├── node1_main.cpp
    ├── node2.cpp
    ├── node2.hpp
    ├── node2_main.cpp
    ├── node3.cpp
    ├── node3.hpp
    ├── node3_main.cpp
    └── sample_class.cpp

4 directories, 13 files
```
            


#### Create a dedicated package for interfaces with dependencies


```commandline
notros2 pkg create ac_test_package6 interfaces_only --ament-dependencies geometry_msgs
```

that outputs

```commandline
Creating interfaces_only (ament_cmake) package ac_test_package6 ... 
Creating package.xml... 
Creating CMakeLists.txt... 
Adding CMakeLists.txt directive for interfaces only package ...
Creating sample msg ...
Creating sample srv ...
```
           
and a folder structure

```commandline
ac_test_package6
├── CMakeLists.txt
├── msg
│   └── AmazingQuote.msg
├── package.xml
└── srv
    └── ReviewAQuote.srv

3 directories, 4 files
```
            

