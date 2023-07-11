# (Murilo's) NOT ros2 Utilities 

These are utilities NOT for ros2.

It creates a command line entry point called `notros2`.

Get information with

```commandline
notros2 -h
```

### `notros2` package utilities

```commandline
notros2 pkg -h
```

### Set default information for generating package templates

```commandline
notros2 pkg set -h
```

### Create package templates

```commandline
notros2 pkg create -h
```

## Example

```commandline
notros2 pkg create test_package ament_cmake --ament-dependencies rclcpp std_msgs --add-nodes node1 node2 node3 --has-library
```
