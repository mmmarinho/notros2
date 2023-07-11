def get_source(node_name: str, NodeName: str) -> str:
    return f"""\
//Template from https://ros2-tutorial.readthedocs.io/en/latest/
#include <rclcpp/rclcpp.hpp>

#include "{node_name}.hpp"

int main(int argc, char** argv)
{{
    rclcpp::init(argc,argv);

    try
    {{
        auto node = std::make_shared<{NodeName}>();

        rclcpp::spin(node);
    }}
    catch (const std::exception& e)
    {{
        std::cerr << std::string("::Exception::") << e.what();
    }}

    return 0;
}}
"""
