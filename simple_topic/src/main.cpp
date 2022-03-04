/*****************************************************************************
*
* Copyright 2020 RoboticsPlus Ltd
*
*****************************************************************************/

#include "rclcpp/rclcpp.hpp"
#include <thread>
#include "simple.h"


using namespace robot;


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto simple_node = std::make_shared<SimpleNode>("simple_node");

    rclcpp::spin(simple_node);
    rclcpp::shutdown();

    return 0;
}

