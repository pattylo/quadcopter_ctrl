#include "quadcopter_ctrl/ctrl_server.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <stdexcept>

namespace
{
constexpr double kTwoPi = 6.28318530717958647692;
}

ctrl_server::ctrl_server(
    std::shared_ptr<rclcpp::Node> node
): _node(node)
{
    std::cout << "=== GIMBAL CTRL DRIVER INIT ===" << std::endl;
    std::cout << "NODE NAME: " << this->_node->get_name() << std::endl;    

    hover_z = _node->declare_parameter<double>("hover_z", 1.5);
    if (hover_z <= 0.0) {
        throw std::invalid_argument("hover_z must be greater than 0.0 metres");
    }
    load_trajectory_parameters();

    arming_client = _node->create_client<mavros_msgs::srv::CommandBool>(
        "/mavros/cmd/arming"
    );

    set_mode_client = _node->create_client<mavros_msgs::srv::SetMode>(
        "/mavros/set_mode"
    );        
    rclcpp::on_shutdown([this]() {
        RCLCPP_WARN(_node->get_logger(), "CTRL + C DETECTED -> LAND");
        land();
    });

    auto qos_mavros_pose =
    rclcpp::QoS(rclcpp::KeepLast(10))
        .best_effort()
        .durability_volatile();

    pose_sub = _node->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/mavros/local_position/pose",
        qos_mavros_pose,
        std::bind(&ctrl_server::pose_callback, this, std::placeholders::_1)
    );

    mavros_state_sub = _node->create_subscription<mavros_msgs::msg::State>(
        "/mavros/state",
        10,
        std::bind(&ctrl_server::state_callback, this, std::placeholders::_1)
    );

    pose_desired_pub = _node->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/mavros/setpoint_position/local",
        10
    );

    set_prerequisite();

    heartbeat_timer = _node->create_wall_timer(
        std::chrono::milliseconds(50), // 20 Hz -> 1000 ms/20 = 50 ms
        std::bind(&ctrl_server::heartbeat_timer_callback, this)
    );
}

ctrl_server::~ctrl_server()
{

}

void ctrl_server::heartbeat_timer_callback()
{
    if (pose_starto && state_starto)
    {
        this->fsm_manager();
        this->ctrl_pub();
    }
}

void ctrl_server::fsm_manager()
{
    switch (fsm)
    {
    case IDLE:
        if (get_ready())
        {
            fsm = ARMED;
            last_request = get_clock_time();
        }
            
        break;

    case ARMED:        
        if (taking_off())
        {
            fsm = TOOKOFF;
            last_request = get_clock_time();
        }            
        
        break;

    case TOOKOFF:
        RCLCPP_INFO(_node->get_logger(), "TAKEOVER/HOVER SUCCESS!");
        trajectory_start_time = get_clock_time();
        trajectory_complete = false;
        fsm = MOVE;
        break;

    case MOVE:
        update_trajectory(get_clock_time() - trajectory_start_time);
        if (trajectory_complete && trajectory_auto_land && !land_requested)
        {
            RCLCPP_INFO(_node->get_logger(), "Trajectory complete; requesting AUTO.LAND");
            land_requested = true;
            land();
            fsm = LAND;
        }
        break;

    case LAND:
        break;
    
    default:
        break;
    }

}

void ctrl_server::load_trajectory_parameters()
{
    trajectory_name = _node->declare_parameter<std::string>("trajectory.type", "circle");
    std::transform(
        trajectory_name.begin(), trajectory_name.end(), trajectory_name.begin(),
        [](unsigned char character) { return static_cast<char>(std::tolower(character)); });

    trajectory_repeat_count = _node->declare_parameter<int>("trajectory.repeat_count", 1);
    trajectory_start_delay = _node->declare_parameter<double>("trajectory.start_delay", 1.0);
    trajectory_z_offset = _node->declare_parameter<double>("trajectory.z_offset", 0.0);
    trajectory_auto_land = _node->declare_parameter<bool>("trajectory.auto_land", false);

    circle_radius = _node->declare_parameter<double>("circle.radius", 1.0);
    circle_period = _node->declare_parameter<double>("circle.period", 8.0);
    circle_center_x = _node->declare_parameter<double>("circle.center_x", -1.0);
    circle_center_y = _node->declare_parameter<double>("circle.center_y", 0.0);
    circle_clockwise = _node->declare_parameter<bool>("circle.clockwise", false);

    lemniscate_amplitude_x =
        _node->declare_parameter<double>("lemniscate.amplitude_x", 1.0);
    lemniscate_amplitude_y =
        _node->declare_parameter<double>("lemniscate.amplitude_y", 0.5);
    lemniscate_period = _node->declare_parameter<double>("lemniscate.period", 8.0);
    lemniscate_center_x = _node->declare_parameter<double>("lemniscate.center_x", 0.0);
    lemniscate_center_y = _node->declare_parameter<double>("lemniscate.center_y", 0.0);

    square_side_length = _node->declare_parameter<double>("square.side_length", 1.0);
    square_speed = _node->declare_parameter<double>("square.speed", 0.5);
    square_center_x = _node->declare_parameter<double>("square.center_x", 0.5);
    square_center_y = _node->declare_parameter<double>("square.center_y", 0.5);

    step_x = _node->declare_parameter<double>("step.x", 1.0);
    step_y = _node->declare_parameter<double>("step.y", 0.0);
    step_z = _node->declare_parameter<double>("step.z", 0.0);
    step_hold_time = _node->declare_parameter<double>("step.hold_time", 2.0);

    if (trajectory_name == "circle")
    {
        trajectory_type = TrajectoryType::CIRCLE;
    }
    else if (
        trajectory_name == "lemniscate" || trajectory_name == "heminiscate" ||
        trajectory_name == "figure_eight")
    {
        trajectory_type = TrajectoryType::LEMNISCATE;
    }
    else if (trajectory_name == "square")
    {
        trajectory_type = TrajectoryType::SQUARE;
    }
    else if (trajectory_name == "step")
    {
        trajectory_type = TrajectoryType::STEP;
    }
    else
    {
        throw std::invalid_argument(
            "trajectory.type must be circle, lemniscate, square, or step");
    }

    if (trajectory_repeat_count < 1)
    {
        throw std::invalid_argument("trajectory.repeat_count must be at least 1");
    }
    if (trajectory_start_delay < 0.0)
    {
        throw std::invalid_argument("trajectory.start_delay cannot be negative");
    }
    if (circle_radius <= 0.0 || circle_period <= 0.0)
    {
        throw std::invalid_argument("circle.radius and circle.period must be positive");
    }
    if (
        lemniscate_amplitude_x <= 0.0 || lemniscate_amplitude_y <= 0.0 ||
        lemniscate_period <= 0.0)
    {
        throw std::invalid_argument(
            "lemniscate amplitudes and lemniscate.period must be positive");
    }
    if (square_side_length <= 0.0 || square_speed <= 0.0)
    {
        throw std::invalid_argument("square.side_length and square.speed must be positive");
    }
    if (step_hold_time <= 0.0)
    {
        throw std::invalid_argument("step.hold_time must be positive");
    }

    RCLCPP_INFO(
        _node->get_logger(), "Configured %s trajectory (%d repetition%s, auto_land=%s)",
        trajectory_name.c_str(), trajectory_repeat_count,
        trajectory_repeat_count == 1 ? "" : "s", trajectory_auto_land ? "true" : "false");
}

double ctrl_server::trajectory_cycle_duration() const
{
    switch (trajectory_type)
    {
    case TrajectoryType::CIRCLE:
        return circle_period;
    case TrajectoryType::LEMNISCATE:
        return lemniscate_period;
    case TrajectoryType::SQUARE:
        return 4.0 * square_side_length / square_speed;
    case TrajectoryType::STEP:
        return 2.0 * step_hold_time;
    }
    return 1.0;
}

void ctrl_server::update_trajectory(double elapsed_seconds)
{
    if (elapsed_seconds < trajectory_start_delay)
    {
        pose_desired.pose.position.x = hover_start_pt.x();
        pose_desired.pose.position.y = hover_start_pt.y();
        pose_desired.pose.position.z = hover_start_pt.z();
        return;
    }

    const double cycle_duration = trajectory_cycle_duration();
    const double trajectory_elapsed = elapsed_seconds - trajectory_start_delay;
    const double total_duration = cycle_duration * trajectory_repeat_count;
    const bool just_completed = !trajectory_complete && trajectory_elapsed >= total_duration;
    trajectory_complete = trajectory_elapsed >= total_duration;

    // All four paths finish at their start point. Pinning the phase to zero after
    // completion prevents a timer overrun from leaving a small endpoint error.
    const double cycle_elapsed = trajectory_complete
        ? 0.0
        : std::fmod(std::max(0.0, trajectory_elapsed), cycle_duration);

    double relative_x = 0.0;
    double relative_y = 0.0;
    double relative_z = trajectory_z_offset;

    switch (trajectory_type)
    {
    case TrajectoryType::CIRCLE:
    {
        double theta = kTwoPi * cycle_elapsed / circle_period;
        if (circle_clockwise)
        {
            theta = -theta;
        }
        relative_x = circle_center_x + circle_radius * std::cos(theta);
        relative_y = circle_center_y + circle_radius * std::sin(theta);
        break;
    }
    case TrajectoryType::LEMNISCATE:
    {
        const double theta = kTwoPi * cycle_elapsed / lemniscate_period;
        relative_x = lemniscate_center_x + lemniscate_amplitude_x * std::sin(theta);
        relative_y = lemniscate_center_y + lemniscate_amplitude_y * std::sin(2.0 * theta);
        break;
    }
    case TrajectoryType::SQUARE:
    {
        const double half_side = 0.5 * square_side_length;
        const double distance = cycle_elapsed * square_speed;
        if (distance < square_side_length)
        {
            relative_x = -half_side + distance;
            relative_y = -half_side;
        }
        else if (distance < 2.0 * square_side_length)
        {
            relative_x = half_side;
            relative_y = -half_side + (distance - square_side_length);
        }
        else if (distance < 3.0 * square_side_length)
        {
            relative_x = half_side - (distance - 2.0 * square_side_length);
            relative_y = half_side;
        }
        else
        {
            relative_x = -half_side;
            relative_y = half_side - (distance - 3.0 * square_side_length);
        }
        relative_x += square_center_x;
        relative_y += square_center_y;
        break;
    }
    case TrajectoryType::STEP:
        if (cycle_elapsed < step_hold_time && !trajectory_complete)
        {
            relative_x = step_x;
            relative_y = step_y;
            relative_z += step_z;
        }
        break;
    }

    pose_desired.pose.position.x = hover_start_pt.x() + relative_x;
    pose_desired.pose.position.y = hover_start_pt.y() + relative_y;
    pose_desired.pose.position.z = hover_start_pt.z() + relative_z;

    if (just_completed)
    {
        RCLCPP_INFO(
            _node->get_logger(), "%s trajectory completed", trajectory_name.c_str());
    }
}

void ctrl_server::pose_callback(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
    if (!pose_starto)
        pose_starto = true;

    pose_feedback = *msg;
    current_pose_SE3 = posemsg_to_SE3(pose_feedback);
    // std::cout<<cur
}

void ctrl_server::state_callback(mavros_msgs::msg::State::ConstSharedPtr msg)
{
    if (!state_starto)
        state_starto = true;

    mavlink_state = *msg;
}

bool ctrl_server::get_ready()
{
    switch_to_offboard_mode();
    arm();

    return mavlink_state.armed;
}

bool ctrl_server::taking_off()
{
    if (
        Eigen::Vector3d(
            (hover_start_pt - current_pose_SE3.translation())
        ).norm() < 0.1
    )
        return true;
    else
        return false;
}

void ctrl_server::set_prerequisite()
{
    rclcpp::Rate rate(10);
    while (rclcpp::ok()) {
        std::cout<<"hi"<<std::endl;
        rclcpp::spin_some(_node);
        if (pose_starto)
            break;
        rate.sleep();
    }

    last_request = get_clock_time();
    pose_desired.pose.position.x = pose_feedback.pose.position.x;
    pose_desired.pose.position.y = pose_feedback.pose.position.y;
    pose_desired.pose.position.z = hover_z;
    pose_desired.pose.orientation.x = 0.0;
    pose_desired.pose.orientation.y = 0.0;
    pose_desired.pose.orientation.z = 0.0;
    pose_desired.pose.orientation.w = 1.0;

    hover_start_pt = Eigen::Vector3d(
        pose_desired.pose.position.x,
        pose_desired.pose.position.y,
        pose_desired.pose.position.z
    );
    std::cout<<"HOVER AT - \n"<<hover_start_pt<<std::endl;
}

void ctrl_server::ctrl_pub()
{
    pose_desired.header.stamp = _node->get_clock()->now();
    pose_desired.header.frame_id = "map";
    pose_desired_pub->publish(pose_desired);
}

Sophus::SE3d ctrl_server::posemsg_to_SE3(const geometry_msgs::msg::PoseStamped& pose)
{
    return Sophus::SE3d(
        Eigen::Quaterniond(
            pose.pose.orientation.w,
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z  
        ).normalized().toRotationMatrix(),
        Eigen::Translation3d(
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z
        ).translation()
    );
}

void ctrl_server::arm()
{
    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = true;

    arming_client->async_send_request(
        request,
        [this](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) 
        {
            if (future.get()->success) 
                output_okay_shit("ARM ACCEPTED");
            else 
                output_bad_shit("ARM REJECTED");
        }
    );
}

void ctrl_server::switch_to_offboard_mode()
{
    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->base_mode = 0;
    request->custom_mode = "OFFBOARD";

    set_mode_client->async_send_request(
        request,
        [this](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) 
        {
            if (future.get()->mode_sent) 
                output_okay_shit("OFFBOARD ACCEPTED");
            else 
                output_bad_shit("OFFBOARD REJECTED");
        }
    );
}

void ctrl_server::land()
{
    auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    req->custom_mode = "AUTO.LAND";
    set_mode_client->async_send_request(req);
}
