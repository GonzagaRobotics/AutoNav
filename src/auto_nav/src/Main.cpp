#include "Auto.h"

using namespace rclcpp;

int main(int argc, char *argv[])
{
    init(argc, argv);

    spin(std::make_shared<AutoNav>());

    shutdown();

    return 0;
}