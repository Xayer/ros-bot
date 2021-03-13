#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"

class NumberCounterNode : public rclcpp::Node
{
public:
    NumberCounterNode() : Node("number_counter")
    {
        publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number_count", 10);
        subscriber_ = this->create_subscription<example_interfaces::msg::Int64>("number", 10,
            std::bind(&NumberCounterNode::recieveNumber, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
                                        std::bind(&NumberCounterNode::publishNumber, this));
        RCLCPP_INFO(this->get_logger(), "Number publisher has been started");
    }

private:
    std::int64_t counter = 0;
    void publishNumber()
    {
        auto msg = example_interfaces::msg::Int64();
        msg.data = counter;
        publisher_->publish(msg);
    }

    void recieveNumber(example_interfaces::msg::Int64::SharedPtr topicData) {
        counter = counter + topicData->data;
    }
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberCounterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}