#ifndef CTRL_SERVER_H
#define CTRL_SERVER_H

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>

#include <eigen3/Eigen/Dense>

#define IDLE 0
#define ARMED 1
#define TOOKOFF 2
#define MOVE 3
#define LAND 4

class ctrl_server
{
    private:
        enum class TrajectoryType
        {
            HOVER,
            CIRCLE,
            LEMNISCATE,
            SQUARE,
            STEP,
            MULTI_WP
        };

        // ROS
        std::shared_ptr<rclcpp::Node> _node;
        // fsm
        rclcpp::TimerBase::SharedPtr heartbeat_timer;
        rclcpp::PreShutdownCallbackHandle shutdown_callback;
        int fsm = IDLE;
        bool pose_starto = false;
        bool state_starto = false;
        double last_request = 0.0;
        Eigen::Vector3d hover_start_pt;
        double takeoff_z = 1.5;

        // Trajectory parameters. Relative planar offsets use hover_start_pt.
        TrajectoryType trajectory_type = TrajectoryType::HOVER;
        std::string trajectory_name = "hover";
        int trajectory_repeat_count = 1;
        double trajectory_start_delay = 1.0;
        bool trajectory_auto_land = false;

        double circle_radius = 1.0;
        double circle_period = 8.0;
        double circle_center_x = -1.0;
        double circle_center_y = 0.0;
        bool circle_clockwise = false;

        double lemniscate_amplitude_x = 1.0;
        double lemniscate_amplitude_y = 0.5;
        double lemniscate_period = 8.0;
        double lemniscate_center_x = 0.0;
        double lemniscate_center_y = 0.0;

        double square_side_length = 1.0;
        double square_speed = 0.5;
        double square_center_x = 0.5;
        double square_center_y = 0.5;

        double step_x = 1.0;
        double step_y = 0.0;
        double step_z = 0.0;
        double step_hold_time = 2.0;

        std::string multi_wp_frame = "relative";
        double multi_wp_z_fix = 1.5;
        std::vector<double> multi_wp_waypoints;
        double multi_wp_reach_tolerance = 0.1;
        double multi_wp_hold_time = 0.0;
        std::size_t multi_wp_index = 0;
        int multi_wp_completed_passes = 0;
        double multi_wp_reached_time = -1.0;

        double trajectory_start_time = 0.0;
        bool trajectory_complete = false;

        void heartbeat_timer_callback();
        void fsm_manager();
        void load_trajectory_parameters();
        void update_trajectory(double elapsed_seconds);
        void update_multi_wp(double elapsed_seconds);
        double trajectory_cycle_duration() const;
        bool get_ready();
        bool taking_off();
        void set_prerequisite();
        void arm();
        void switch_to_offboard_mode();
        void land();

        // feedback
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;
        geometry_msgs::msg::PoseStamped pose_feedback;

        void pose_callback(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);

        rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr mavros_state_sub;
        mavros_msgs::msg::State mavlink_state;

        void state_callback(mavros_msgs::msg::State::ConstSharedPtr msg);

        // ctrl
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_desired_pub;
        geometry_msgs::msg::PoseStamped pose_desired;

        void ctrl_pub();

        // mavlink client
        rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client;
        rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client;

        double get_clock_time() const { return _node->get_clock()->now().seconds(); }
        inline void output_okay_shit(const std::string & lala)
        {
            RCLCPP_INFO(_node->get_logger(), "%s", lala.c_str());
        }
        inline void output_bad_shit(const std::string & lala)
        {
            RCLCPP_INFO(_node->get_logger(), "%s", lala.c_str());
        }
    public:
        ctrl_server(std::shared_ptr<rclcpp::Node> node);
        ~ctrl_server();
};

#endif
