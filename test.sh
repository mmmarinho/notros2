#!/usr/bin/env bash
set -e

mkdir tmp
cd tmp

# Set
notros2 pkg set -h
notros2 pkg set --mantainer-name "John Testerson"
notros2 pkg set --mantainer-email "john.testerson@testcorp.info"
notros2 pkg set --license "Licenest"

notros2 pkg create -h
# Create simplest package
notros2 pkg create test_package1 ament_cmake
# Create simple package with dependencies
notros2 pkg create test_package2 ament_cmake --ament-dependencies rclcpp geometry_msgs
# library only
notros2 pkg create test_package3 ament_cmake --ament-dependencies rclcpp geometry_msgs --has-library
# nodes only
notros2 pkg create test_package4 ament_cmake --ament-dependencies rclcpp geometry_msgs --add-nodes node1 node2
# Several nodes and a library
notros2 pkg create test_package5 ament_cmake \
--ament-dependencies rclcpp geometry_msgs \
--add-nodes node1 node2 node3 \
--has-library
# Interfaces only
notros2 pkg create test_package6 interfaces_only --ament-dependencies rclcpp geometry_msgs
