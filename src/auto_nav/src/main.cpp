#include "auto_nav.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<AutoNav>());

    rclcpp::shutdown();

    return 0;
}