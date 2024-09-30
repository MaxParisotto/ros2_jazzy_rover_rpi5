#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"  // Assuming the motor commands are sent as an array of integers

class MotorCommandPublisher : public rclcpp::Node {
public:
    MotorCommandPublisher() : Node("motor_command_publisher") {
        // Creating a publisher that will publish on the 'motor_commands' topic
        publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("motor_commands", 10);

        // Timer to repeatedly publish motor commands
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), std::bind(&MotorCommandPublisher::publish_motor_commands, this));
    }

private:
    // Method to publish motor commands
    void publish_motor_commands() {
        // Create a message to send
        auto message = std_msgs::msg::Int32MultiArray();
        
        // Populate the message with motor command data (example: two motor commands)
        message.data = {100, 150};  // Example motor speeds for two motors

        // Publish the message
        RCLCPP_INFO(this->get_logger(), "Publishing motor commands: %d, %d", message.data[0], message.data[1]);
        publisher_->publish(message);
    }

    // Publisher object to send motor commands
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_;

    // Timer object to trigger the publishing periodically
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    // Initialize the ROS 2 system
    rclcpp::init(argc, argv);

    // Create and spin the node
    rclcpp::spin(std::make_shared<MotorCommandPublisher>());

    // Shutdown the ROS 2 system
    rclcpp::shutdown();
    return 0;
}