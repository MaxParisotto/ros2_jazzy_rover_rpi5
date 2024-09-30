#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"  // IMU message type from the sensor_msgs package

class IMUSubscriber : public rclcpp::Node {
public:
    IMUSubscriber() : Node("imu_subscriber") {
        // Subscribing to the IMU data topic
        subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu/data_raw", 10, std::bind(&IMUSubscriber::topic_callback, this, std::placeholders::_1));
    }

private:
    // Callback function that gets executed whenever a new IMU message is received
    void topic_callback(const sensor_msgs::msg::Imu::SharedPtr msg) const {
        // Logging the orientation data (as an example)
        RCLCPP_INFO(this->get_logger(), "IMU Orientation [x: %f, y: %f, z: %f, w: %f]", 
                    msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);

        // Log angular velocity data (optional)
        RCLCPP_INFO(this->get_logger(), "Angular velocity [x: %f, y: %f, z: %f]",
                    msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

        // Log linear acceleration data (optional)
        RCLCPP_INFO(this->get_logger(), "Linear acceleration [x: %f, y: %f, z: %f]",
                    msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    }

    // Subscription object to listen to the IMU data topic
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
    // Initialize the ROS 2 system
    rclcpp::init(argc, argv);

    // Create and spin the node
    rclcpp::spin(std::make_shared<IMUSubscriber>());

    // Shutdown the ROS 2 system
    rclcpp::shutdown();
    return 0;
}