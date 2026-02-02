#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>

class CmdVelToRPM : public rclcpp::Node
{
public:
    CmdVelToRPM()
    : Node("cmd_vel_to_rpm")
    {
        // Robot parameters (adjust to your robot)
        wheel_radius_ = 0.10;       // meters
        wheel_separation_ = 0.45;   // meters

        // Publishers for left/right wheel RPM
        left_pub_  = this->create_publisher<std_msgs::msg::Float64>("left_motor_angular_vel", 10);
        right_pub_ = this->create_publisher<std_msgs::msg::Float64>("right_motor_angular_vel", 10);

        // Subscribe to cmd_vel
        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&CmdVelToRPM::cmdVelCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "cmd_vel_to_angular_vel node started.");
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        double v = msg->linear.x;     // forward velocity (m/s)
        double w = msg->angular.z;    // angular velocity (rad/s)

        // Differential drive inverse kinematics
        double v_left  = v - (w * wheel_separation_ / 2.0);
        double v_right = v + (w * wheel_separation_ / 2.0);

        // Convert linear wheel velocity (m/s) → angular velocity (rad/s)
        double omega_left  = v_left  / wheel_radius_;
        double omega_right = v_right / wheel_radius_;

        // Prepare messages
        std_msgs::msg::Float64 left_msg;
        std_msgs::msg::Float64 right_msg;

        left_msg.data  = omega_left;
        right_msg.data = omega_right;

        // Publish results
        left_pub_->publish(left_msg);
        right_pub_->publish(right_msg);
    }

    double wheel_radius_;
    double wheel_separation_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelToRPM>());
    rclcpp::shutdown();
    return 0;
}
