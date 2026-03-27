// ============================================================================
// src/control_planning/src/kart_controller.cpp
// ============================================================================
// ROS 2 Humble lane-following controller for the F1Tenth / Go-Kart.
//
// SUBSCRIBES
//   /perception/lane_guidance   geometry_msgs/Vector3
//     .x  lateral_error_m    + = steer LEFT  (car drifted right)
//     .y  heading_error_rad  + = steer LEFT  (lane curves left)
//     .z  detection_quality  0.0-1.0
//
//   /emergency_stop             std_msgs/Bool
//     true  = stop immediately
//     false = resume
//
// PUBLISHES
//   /drive    ackermann_msgs/AckermannDriveStamped
//
// TUNABLE PARAMETERS (set in vesc_params.yaml or at launch)
//   speed_nominal      (double, m/s)   -- cruise speed while tracking lanes
//   kp_lateral         (double)        -- proportional gain on lateral error
//   kd_lateral         (double)        -- derivative gain on lateral error
//   kp_heading         (double)        -- proportional gain on heading error
//   max_steer_angle    (double, rad)   -- absolute steering clamp
//   min_quality        (double, 0-1)   -- below this, hold last command
// ============================================================================

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

#include <algorithm>   // std::clamp
#include <cmath>

using std::placeholders::_1;

using Vector3     = geometry_msgs::msg::Vector3;
using AckermannDS = ackermann_msgs::msg::AckermannDriveStamped;
using BoolMsg     = std_msgs::msg::Bool;

// ============================================================================

class KartController : public rclcpp::Node
{
public:
    KartController() : Node("kart_controller")
    {
        // -- Declare tunable ROS parameters ----------------------------------
        this->declare_parameter("speed_nominal", 1.0);
        this->declare_parameter("kp_lateral", 0.8);
        this->declare_parameter("kd_lateral", 0.05);
        this->declare_parameter("kp_heading", 0.5);
        this->declare_parameter("max_steer_angle", 0.34);   // ~20 degrees
        this->declare_parameter("min_quality", 0.4);

        // -- Subscribers -----------------------------------------------------
        guidance_sub_ = this->create_subscription<Vector3>(
            "/perception/lane_guidance", 10,
            std::bind(&KartController::guidanceCallback, this, _1));

        estop_sub_ = this->create_subscription<BoolMsg>(
            "/emergency_stop", 10,
            std::bind(&KartController::estopCallback, this, _1));

        // -- Publisher -------------------------------------------------------
        drive_pub_ = this->create_publisher<AckermannDS>("/drive", 10);

        RCLCPP_INFO(this->get_logger(),
            "KartController ready. Waiting for /perception/lane_guidance ...");
    }

private:

    // -- State ---------------------------------------------------------------
    bool         estop_active_   = false;
    double       prev_lat_error_ = 0.0;
    rclcpp::Time prev_time_;
    bool         have_prev_time_ = false;

    // -- ROS handles ---------------------------------------------------------
    rclcpp::Subscription<Vector3>::SharedPtr  guidance_sub_;
    rclcpp::Subscription<BoolMsg>::SharedPtr  estop_sub_;
    rclcpp::Publisher<AckermannDS>::SharedPtr drive_pub_;

    // -- Emergency stop callback ---------------------------------------------
    void estopCallback(const BoolMsg::SharedPtr msg)
    {
        estop_active_ = msg->data;
        if (estop_active_) {
            publishStop();
            RCLCPP_WARN(this->get_logger(), "!!! EMERGENCY STOP ACTIVE !!!");
        } else {
            RCLCPP_INFO(this->get_logger(), "Emergency stop released. Resuming.");
        }
    }

    // -- Main guidance callback ----------------------------------------------
    void guidanceCallback(const Vector3::SharedPtr msg)
    {
        if (estop_active_) {
            publishStop();
            return;
        }

        const double lat_err = msg->x;
        const double hdg_err = msg->y;
        const double quality = msg->z;

        // Skip frames where detection is too uncertain
        const double min_q = this->get_parameter("min_quality").as_double();
        if (quality < min_q) {
            RCLCPP_DEBUG(this->get_logger(),
                "Low quality (%.2f < %.2f) -- holding last command.", quality, min_q);
            return;
        }

        const double steer = computeControl(lat_err, hdg_err);

        // Publish AckermannDriveStamped
        AckermannDS drive_msg;
        drive_msg.header.stamp = this->now();
        drive_msg.drive.steering_angle = static_cast<float>(steer);
        drive_msg.drive.speed = static_cast<float>(
            this->get_parameter("speed_nominal").as_double());
        drive_pub_->publish(drive_msg);
    }

    // -- Control algorithm ---------------------------------------------------
    double computeControl(double lat_err, double hdg_err)
    {
        const double kp_lat = this->get_parameter("kp_lateral").as_double();
        const double kd_lat = this->get_parameter("kd_lateral").as_double();
        const double kp_hdg = this->get_parameter("kp_heading").as_double();
        const double max_st = this->get_parameter("max_steer_angle").as_double();

        // Derivative of lateral error
        double d_lat = 0.0;
        rclcpp::Time now = this->now();
        if (have_prev_time_) {
            double dt = (now - prev_time_).seconds();
            if (dt > 1e-6 && dt < 0.5) {
                d_lat = (lat_err - prev_lat_error_) / dt;
            }
        }
        prev_lat_error_ = lat_err;
        prev_time_      = now;
        have_prev_time_  = true;

        // PD on lateral + P on heading
        double steer = kp_lat * lat_err + kd_lat * d_lat + kp_hdg * hdg_err;

        // Clamp
        steer = std::clamp(steer, -max_st, max_st);
        return steer;
    }

    // -- Publish zero-speed stop command -------------------------------------
    void publishStop()
    {
        AckermannDS stop;
        stop.header.stamp = this->now();
        stop.drive.speed = 0.0f;
        stop.drive.steering_angle = 0.0f;
        drive_pub_->publish(stop);
    }
};

// ============================================================================

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KartController>());
    rclcpp::shutdown();
    return 0;
}
