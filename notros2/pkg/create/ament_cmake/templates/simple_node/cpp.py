def get_source(node_name:str, NodeName:str) -> str:
    return f"""\
#include "{node_name}.hpp"

/**
 * @brief {NodeName}::{NodeName} Default constructor.
 */
{NodeName}::{NodeName}():
    rclcpp::Node("{node_name}"),
    timer_period_(0.5),
    print_count_(0)
{{
    timer_ = create_wall_timer(
                std::chrono::milliseconds(long(timer_period_*1e3)),
                std::bind(&{NodeName}::_timer_callback, this)
                );
}}

/**
 * @brief {NodeName}::_timer_callback periodically prints class info using RCLCPP_INFO.
 */
void {NodeName}::_timer_callback()
{{
    RCLCPP_INFO_STREAM(get_logger(),
                       std::string("Printed ") +
                       std::to_string(print_count_) +
                       std::string(" times.")
                       );
    print_count_++;
}}
"""