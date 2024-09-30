#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"  // Assuming the encoder data is published as an array of integers

class EncoderSubscriber : public rclcpp::Node {
public:
    EncoderSubscriber() : Node("encoder_subscriber") {
        // Subscribing to the topic where encoder data is published
        subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "motor_encoders", 10, std::bind(&EncoderSubscriber::topic_callback, this, std::placeholders::_1));
    }

private:
    // Callback function to handle the incoming encoder data
    void topic_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) const {
        // Log the encoder data to the console
        RCLCPP_INFO(this->get_logger(), "Received encoder data:");
        for (auto value : msg->data) {
            RCLCPP_INFO(this->get_logger(), "Encoder value: %d", value);
        }
    }

    // Subscription object to handle the subscribed topic
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
    // Initialize the ROS 2 system
    rclcpp::init(argc, argv);

    // Create and spin the node
    rclcpp::spin(std::make_shared<EncoderSubscriber>());

    // Shutdown the ROS 2 system
    rclcpp::shutdown();
    return 0;
}