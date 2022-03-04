#ifndef SIMPLE_H
#define SIMPLE_H

#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include "std_msgs/msg/u_int32.hpp"
#include <vector>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <memory>


namespace robot
{
    class SimpleNode : public rclcpp::Node
    {
        using node_start_req_t = std_msgs::msg::UInt32;

    public:
        SimpleNode(const std::string& node_name);
        virtual ~SimpleNode();

    private:  // Functions
        void node_start_req_callback(const node_start_req_t::SharedPtr msg);

    private:  // Variables
        rclcpp::Subscription<node_start_req_t>::SharedPtr m_node_start_req_subscription;
    };  // class SimpleNode

}  // namespace robot

#endif  // SIMPLE_H
