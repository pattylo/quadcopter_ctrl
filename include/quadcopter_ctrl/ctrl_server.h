#ifndef CTRL_SERVER_H
#define CTRL_SERVER_H

#include <chrono>
#include <string>
#include <memory>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstdint>
#include <fstream>
#include <iomanip>

#include "rclcpp/rclcpp.hpp"
#include <sophus/se3.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>

#include <eigen3/Eigen/Dense>

#include <memory>
#include <deque>

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
            CIRCLE,
            LEMNISCATE,
            SQUARE,
            STEP
        };

        // ROS
        std::shared_ptr<rclcpp::Node> _node;
        rclcpp::QoS qos_live = rclcpp::QoS(rclcpp::KeepLast(1))
            .best_effort()
            .durability_volatile();

        // fsm
        rclcpp::TimerBase::SharedPtr heartbeat_timer;                
        int fsm = IDLE;
        bool pose_starto = false;
        bool state_starto = false;
        double last_request;
        Eigen::Vector3d hover_start_pt;
        double hover_z = 1.5;
        Sophus::SE3d current_pose_SE3;

        // Trajectory parameters. Planar offsets are relative to hover_start_pt.
        TrajectoryType trajectory_type = TrajectoryType::CIRCLE;
        std::string trajectory_name = "circle";
        int trajectory_repeat_count = 1;
        double trajectory_start_delay = 1.0;
        double trajectory_z_offset = 0.0;
        bool trajectory_auto_land = false;

        double circle_radius = 1.0;
        double circle_period = 8.0;
        double circle_center_x = -1.0;
        double circle_center_y = 0.0;
        bool circle_clockwise = false;

        double lemniscate_amplitude_x = 1.0;
        double lemniscate_amplitude_y = 1.0;
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

        double trajectory_start_time = 0.0;
        bool trajectory_complete = false;
        bool land_requested = false;

        void heartbeat_timer_callback();
        void fsm_manager();
        void load_trajectory_parameters();
        void update_trajectory(double elapsed_seconds);
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
        rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture pending_set_mode_fut;
        rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture pending_arm_fut;
        
        // helper
        rclcpp::Time start_time;

        Sophus::SE3d posemsg_to_SE3(const geometry_msgs::msg::PoseStamped& pose);
        inline double get_clock_time(){ return _node->get_clock()->now().seconds(); }
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
