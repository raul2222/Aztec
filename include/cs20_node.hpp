#ifndef CS20_NODE_HPP_
#define CS20_NODE_HPP_

#include <rclcpp/clock.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/time_source.hpp>

namespace cs20_ros{


class CS20_ROS_PUBLIC cs20_node : public rclcpp::Node
{
    public:
    explicit cs20_node(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    virtual ~cs20_node();

    void publish_scan(const double scan_time, ResponseNodeArray nodes, size_t node_count);

    /* service callbacks */
    //void stop_motor(const EmptyRequest req, EmptyResponse res);
    //void start_motor(const EmptyRequest req, EmptyResponse res);

    private:

    bool cs20_Config();
    void publish_loop();

    void calculate_framerate();

    /* parameters */
    std::string channel_type_;
    std::string tcp_ip_;
    std::string serial_port_;
    std::string topic_name_;

        /* Timer */
    Timer m_timer;


}

}