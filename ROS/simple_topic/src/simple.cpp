#include "simple.h"
#include "s_log.h"

// C library headers
#include <stdio.h>
#include <string.h>


namespace robot
{
    using namespace std;
    using std::placeholders::_1;


    SimpleNode::SimpleNode(const std::string& node_name) : Node(node_name)
    {
        // Publishers, subscribers and services
        m_node_start_req_subscription = create_subscription<node_start_req_t>("simple_topic/simple_topic_node", rclcpp::QoS(10),
                                                std::bind(&SimpleNode::node_start_req_callback,
                                                this, std::placeholders::_1));
       SLog::log_info("Started Default ROS Node");
    }


    SimpleNode::~SimpleNode()
    {
        SLog::log_info("Default ROS Node terminated gracefully...");
    }

    void SimpleNode::node_start_req_callback(const node_start_req_t::SharedPtr msg)
    {
        uint32_t msg_data = msg->data;
        SLog::log_info("SimpleNode::node_start_req_callback - Requested to start with data " + 
                        std::to_string(msg_data));
    }

} // namespace robot