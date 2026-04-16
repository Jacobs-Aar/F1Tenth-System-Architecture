// ============================================================================
// src/control_planning/src/kart_controller.cpp
// ============================================================================
// v2.1
//
// CHANGES FROM v2.0
//   - `drive_topic` is now a parameter. Default "ackermann_cmd" (not "/drive"),
//     matching what vesc_ackermann's AckermannToVesc node subscribes to. v2.0
//     published to "/drive" which wasn't wired to anything → no steering.
//
// CONTROL MODES (param `controller_mode`)
//   "pd"            — subscribe /perception/lane_guidance, PD on lateral +
//                     P on heading.
//   "pure_pursuit"  — subscribe /perception/pursuit_target (car-frame point
//                     at fixed lookahead). Ackermann steer via:
//                       δ = atan( 2·L·sin(α) / Ld )
// ============================================================================

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

#include <algorithm>
#include <cmath>
#include <string>

using std::placeholders::_1;
using Vector3     = geometry_msgs::msg::Vector3;
using AckermannDS = ackermann_msgs::msg::AckermannDriveStamped;
using BoolMsg     = std_msgs::msg::Bool;

class KartController : public rclcpp::Node
{
public:
    KartController() : Node("kart_controller")
    {
        // --- parameters ---
        this->declare_parameter<std::string>("controller_mode", "pure_pursuit");
        this->declare_parameter<std::string>("drive_topic",     "ackermann_cmd");
        this->declare_parameter("speed_nominal",   1.0);
        this->declare_parameter("wheelbase",       0.33);
        this->declare_parameter("max_steer_angle", 0.40);
        this->declare_parameter("min_quality",     0.4);
        this->declare_parameter("kp_lateral", 2.0);
        this->declare_parameter("kd_lateral", 0.08);
        this->declare_parameter("kp_heading", 1.5);

        const auto drive_topic = this->get_parameter("drive_topic").as_string();

        // --- subs ---
        estop_sub_ = this->create_subscription<BoolMsg>(
            "/emergency_stop", 10,
            std::bind(&KartController::estopCallback, this, _1));
        guidance_sub_ = this->create_subscription<Vector3>(
            "/perception/lane_guidance", 10,
            std::bind(&KartController::guidanceCallback, this, _1));
        pursuit_sub_ = this->create_subscription<Vector3>(
            "/perception/pursuit_target", 10,
            std::bind(&KartController::pursuitCallback, this, _1));

        // --- pub ---
        drive_pub_ = this->create_publisher<AckermannDS>(drive_topic, 10);

        const auto mode = this->get_parameter("controller_mode").as_string();
        RCLCPP_INFO(this->get_logger(),
            "KartController v2.1 ready | mode=%s drive_topic=%s "
            "wheelbase=%.3fm max_steer=%.3frad",
            mode.c_str(), drive_topic.c_str(),
            this->get_parameter("wheelbase").as_double(),
            this->get_parameter("max_steer_angle").as_double());
    }

private:
    bool   estop_active_   = false;
    double prev_lat_err_   = 0.0;
    rclcpp::Time prev_time_;
    bool   have_prev_time_ = false;

    rclcpp::Subscription<Vector3>::SharedPtr  guidance_sub_;
    rclcpp::Subscription<Vector3>::SharedPtr  pursuit_sub_;
    rclcpp::Subscription<BoolMsg>::SharedPtr  estop_sub_;
    rclcpp::Publisher<AckermannDS>::SharedPtr drive_pub_;

    void estopCallback(const BoolMsg::SharedPtr msg) {
        estop_active_ = msg->data;
        if (estop_active_) {
            publishStop();
            RCLCPP_WARN(this->get_logger(), "!!! EMERGENCY STOP ACTIVE !!!");
        } else {
            RCLCPP_INFO(this->get_logger(), "E-stop released.");
        }
    }

    void guidanceCallback(const Vector3::SharedPtr msg) {
        if (estop_active_) { publishStop(); return; }
        const auto mode = this->get_parameter("controller_mode").as_string();
        if (mode != "pd") return;

        const double min_q = this->get_parameter("min_quality").as_double();
        if (msg->z < min_q) return;

        const double lat = msg->x;
        const double hdg = msg->y;
        const double kp_lat = this->get_parameter("kp_lateral").as_double();
        const double kd_lat = this->get_parameter("kd_lateral").as_double();
        const double kp_hdg = this->get_parameter("kp_heading").as_double();
        const double max_st = this->get_parameter("max_steer_angle").as_double();

        double d_lat = 0.0;
        rclcpp::Time now = this->now();
        if (have_prev_time_) {
            double dt = (now - prev_time_).seconds();
            if (dt > 1e-6 && dt < 0.5) d_lat = (lat - prev_lat_err_) / dt;
        }
        prev_lat_err_ = lat;
        prev_time_ = now;
        have_prev_time_ = true;

        double steer = kp_lat * lat + kd_lat * d_lat + kp_hdg * hdg;
        steer = std::clamp(steer, -max_st, max_st);
        publishDrive(steer);
    }

    void pursuitCallback(const Vector3::SharedPtr msg) {
        if (estop_active_) { publishStop(); return; }
        const auto mode = this->get_parameter("controller_mode").as_string();
        if (mode != "pure_pursuit") return;

        const double min_q = this->get_parameter("min_quality").as_double();
        if (msg->z < min_q) return;

        const double tx = msg->x;
        const double ty = msg->y;

        const double Ld = std::sqrt(tx*tx + ty*ty);
        if (Ld < 0.05) return;
        if (tx < 0.02) return;   // target behind car

        const double L      = this->get_parameter("wheelbase").as_double();
        const double max_st = this->get_parameter("max_steer_angle").as_double();

        const double alpha = std::atan2(ty, tx);
        double steer = std::atan2(2.0 * L * std::sin(alpha), Ld);
        steer = std::clamp(steer, -max_st, max_st);

        publishDrive(steer);
    }

    void publishDrive(double steer) {
        AckermannDS d;
        d.header.stamp = this->now();
        d.drive.steering_angle = static_cast<float>(steer);
        d.drive.speed = static_cast<float>(
            this->get_parameter("speed_nominal").as_double());
        drive_pub_->publish(d);
    }

    void publishStop() {
        AckermannDS s;
        s.header.stamp = this->now();
        s.drive.speed = 0.0f;
        s.drive.steering_angle = 0.0f;
        drive_pub_->publish(s);
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KartController>());
    rclcpp::shutdown();
    return 0;
}
