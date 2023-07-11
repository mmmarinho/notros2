def get_source(NodeName: str) -> str:
    return f"""\
//Template from https://ros2-tutorial.readthedocs.io/en/latest/
#pragma once

#include <rclcpp/rclcpp.hpp>

/**
 * @brief A ROS2 Node that prints to the console periodically, but in C++.
 */
class {NodeName}: public rclcpp::Node
{{
private:
    double timer_period_;
    int print_count_;
    rclcpp::TimerBase::SharedPtr timer_;

    void _timer_callback();
public:
    {NodeName}();

}};
"""
