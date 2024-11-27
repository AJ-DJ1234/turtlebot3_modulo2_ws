// explore_node.cpp

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <vector>
#include <map>
#include <algorithm>
#include <memory>
#include <string>
#include <chrono>
#include <tf2/LinearMath/Quaternion.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

// Define the ExploreNode class
class ExploreNode : public rclcpp::Node
{
public:
    // Constructor
    ExploreNode();

private:
    // Parameters
    double info_radius_;
    double frontier_visit_threshold_;
    int max_attempts_;
    double timer_interval_;
    int min_frontier_size_;
    int free_threshold_;
    int occupied_threshold_;

    // Subscribers and Publishers
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscriber_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr frontier_pub_;

    // Action Client
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;

    // Transform Listener
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Timer for exploration cycle
    rclcpp::TimerBase::SharedPtr timer_;

    // Latest map data
    nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;

    // Tracking visited frontiers
    std::vector<std::pair<double, double>> visited_frontiers_;
    std::map<std::pair<int, int>, int> frontier_selection_attempts_;

    // Navigation state
    bool navigation_in_progress_ = false;
    std::pair<double, double> current_goal_;

    // Function declarations
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void exploration_cycle();
    void send_goal(const geometry_msgs::msg::PoseStamped::SharedPtr goal_pose);
    geometry_msgs::msg::PoseStamped::SharedPtr get_current_pose();
    std::vector<int> find_frontiers(const nav_msgs::msg::OccupancyGrid &map_data);
    std::vector<std::pair<double, double>> get_frontier_coords(const std::vector<int> &frontier_indices);
    void publish_frontiers(const std::vector<std::pair<double, double>> &frontier_coords);
    std::vector<std::pair<double, double>> filter_frontiers(const std::vector<std::pair<double, double>> &frontier_coords);
    bool is_frontier_visited(const std::pair<double, double> &coord);
    std::pair<double, double> select_frontier(const std::vector<std::pair<double, double>> &frontier_coords, const geometry_msgs::msg::PoseStamped &current_pose);
    double information_gain(const std::pair<double, double> &coord, double radius);
    bool is_goal_position_valid(const geometry_msgs::msg::PoseStamped &goal_pose);
    void mark_frontier_as_invalid(const std::pair<double, double> &coord);
    std::pair<int, int> round_coord(const std::pair<double, double> &coord, int precision = 2);

    // Utility functions
    geometry_msgs::msg::PoseStamped::SharedPtr calculate_goal_pose(
        const std::pair<double, double> &coord,
        const geometry_msgs::msg::PoseStamped &current_pose);
    geometry_msgs::msg::Quaternion yaw_to_quaternion(double yaw);
};

// Constructor definition
ExploreNode::ExploreNode()
    : Node("explore_node"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_)
{
    // Declare parameters with adjusted default values
    this->declare_parameter<double>("info_radius", 1.0);
    this->declare_parameter<double>("frontier_visit_threshold", 0.05); // Reduced to prevent premature exclusion
    this->declare_parameter<int>("max_attempts", 20);
    this->declare_parameter<double>("timer_interval", 2.0);
    this->declare_parameter<int>("min_frontier_size", 1); // Include small frontiers
    this->declare_parameter<int>("free_threshold", 25);    // Adjusted threshold for free cells
    this->declare_parameter<int>("occupied_threshold", 65); // Adjusted threshold for occupied cells

    // Get parameter values
    this->get_parameter("info_radius", info_radius_);
    this->get_parameter("frontier_visit_threshold", frontier_visit_threshold_);
    this->get_parameter("max_attempts", max_attempts_);
    this->get_parameter("timer_interval", timer_interval_);
    this->get_parameter("min_frontier_size", min_frontier_size_);
    this->get_parameter("free_threshold", free_threshold_);
    this->get_parameter("occupied_threshold", occupied_threshold_);

    RCLCPP_INFO(this->get_logger(), "Parameters:");
    RCLCPP_INFO(this->get_logger(), "  info_radius: %.2f", info_radius_);
    RCLCPP_INFO(this->get_logger(), "  frontier_visit_threshold: %.2f", frontier_visit_threshold_);
    RCLCPP_INFO(this->get_logger(), "  max_attempts: %d", max_attempts_);
    RCLCPP_INFO(this->get_logger(), "  timer_interval: %.2f", timer_interval_);
    RCLCPP_INFO(this->get_logger(), "  min_frontier_size: %d", min_frontier_size_);
    RCLCPP_INFO(this->get_logger(), "  free_threshold: %d", free_threshold_);
    RCLCPP_INFO(this->get_logger(), "  occupied_threshold: %d", occupied_threshold_);

    // Initialize map subscriber
    map_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", // Ensure this is the correct topic
        10,
        std::bind(&ExploreNode::map_callback, this, _1)
    );

    // Initialize publisher
    frontier_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("frontiers", 10);

    // Initialize action client
    action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    // Wait for action server
    while (!action_client_->wait_for_action_server(1s)) {
        RCLCPP_INFO(this->get_logger(), "Waiting for Nav2 action server...");
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the action server.");
            return;
        }
    }

    RCLCPP_INFO(this->get_logger(), "Nav2 action server available.");

    // Initialize timer for exploration cycle
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(timer_interval_),
        std::bind(&ExploreNode::exploration_cycle, this)
    );

    RCLCPP_INFO(this->get_logger(), "Exploration node initialized and running...");
}

// Definition of yaw_to_quaternion
geometry_msgs::msg::Quaternion ExploreNode::yaw_to_quaternion(double yaw)
{
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    q.normalize(); // Ensure the quaternion is normalized
    return tf2::toMsg(q);
}

// Definition of calculate_goal_pose
geometry_msgs::msg::PoseStamped::SharedPtr ExploreNode::calculate_goal_pose(
    const std::pair<double, double> &coord,
    const geometry_msgs::msg::PoseStamped &current_pose)
{
    auto goal_pose = std::make_shared<geometry_msgs::msg::PoseStamped>();
    goal_pose->header.frame_id = "map";
    goal_pose->header.stamp = this->get_clock()->now();
    goal_pose->pose.position.x = coord.first;
    goal_pose->pose.position.y = coord.second;
    goal_pose->pose.position.z = 0.0;

    // Calculate orientation towards the goal
    double dx = coord.first - current_pose.pose.position.x;
    double dy = coord.second - current_pose.pose.position.y;
    double yaw = std::atan2(dy, dx);
    goal_pose->pose.orientation = yaw_to_quaternion(yaw);

    return goal_pose;
}

// Callback for map updates
void ExploreNode::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    latest_map_ = msg;
    RCLCPP_DEBUG(this->get_logger(), "Map update received.");

    // Log map statistics
    int total_cells = msg->info.width * msg->info.height;
    int known_cells = 0, free_cells = 0, occupied_cells = 0;
    for (const auto &cell : msg->data)
    {
        if (cell != -1) { // Known cells
            known_cells++;
            if (cell >= 0 && cell <= free_threshold_)
                free_cells++;
            else if (cell >= occupied_threshold_) // Occupied threshold
                occupied_cells++;
        }
    }
    double known_ratio = static_cast<double>(known_cells) / total_cells;
    double free_ratio = static_cast<double>(free_cells) / total_cells;
    double occupied_ratio = static_cast<double>(occupied_cells) / total_cells;

    RCLCPP_INFO(this->get_logger(), "Map Stats - Total: %d, Known: %d (%.2f%%), Free: %d (%.2f%%), Occupied: %d (%.2f%%)",
                total_cells, known_cells, known_ratio * 100.0, free_cells, free_ratio * 100.0, occupied_cells, occupied_ratio * 100.0);
}

// Exploration cycle
void ExploreNode::exploration_cycle()
{
    RCLCPP_INFO(this->get_logger(), "Starting exploration cycle...");

    if (navigation_in_progress_)
    {
        RCLCPP_DEBUG(this->get_logger(), "Navigation is still in progress.");
        return;
    }

    if (latest_map_ == nullptr)
    {
        RCLCPP_DEBUG(this->get_logger(), "Waiting for map data...");
        return;
    }

    auto current_pose = get_current_pose();
    if (current_pose == nullptr)
    {
        RCLCPP_DEBUG(this->get_logger(), "Waiting for current pose...");
        return;
    }

    // Find frontiers
    std::vector<int> frontier_indices = find_frontiers(*latest_map_);
    RCLCPP_INFO(this->get_logger(), "Frontiers found: %zu", frontier_indices.size());

    if (frontier_indices.empty())
    {
        RCLCPP_WARN(this->get_logger(), "No frontiers found. Exploration may be complete or map needs updating.");
        return;
    }

    // Convert indices to coordinates
    std::vector<std::pair<double, double>> frontier_coords = get_frontier_coords(frontier_indices);
    if (frontier_coords.empty())
    {
        RCLCPP_DEBUG(this->get_logger(), "Failed to obtain frontier coordinates.");
        return;
    }

    // Filter frontiers
    std::vector<std::pair<double, double>> filtered_frontiers = filter_frontiers(frontier_coords);
    RCLCPP_INFO(this->get_logger(), "Frontiers after filtering: %zu", filtered_frontiers.size());

    if (filtered_frontiers.empty())
    {
        RCLCPP_INFO(this->get_logger(), "No unvisited frontiers available after filtering.");
        return;
    }

    // Publish frontiers for visualization
    publish_frontiers(filtered_frontiers);

    // Select a frontier
    auto selected_coord = select_frontier(filtered_frontiers, *current_pose);
    if (selected_coord.first == 0.0 && selected_coord.second == 0.0)
    {
        RCLCPP_DEBUG(this->get_logger(), "Failed to select a valid frontier.");
        return;
    }

    // Calculate goal pose
    auto goal_pose = calculate_goal_pose(selected_coord, *current_pose);
    if (goal_pose == nullptr)
    {
        RCLCPP_DEBUG(this->get_logger(), "Failed to calculate goal pose.");
        return;
    }

    // Validate goal position
    if (!is_goal_position_valid(*goal_pose))
    {
        RCLCPP_WARN(this->get_logger(), "Selected goal position is invalid (occupied or out of bounds). Skipping.");
        mark_frontier_as_invalid(selected_coord);
        return;
    }

    // Send goal to navigator
    send_goal(goal_pose);
}

// Function to send goal to Nav2
void ExploreNode::send_goal(const geometry_msgs::msg::PoseStamped::SharedPtr goal_pose)
{
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = *goal_pose;

    RCLCPP_INFO(this->get_logger(), "Sending goal to navigator: x=%.2f, y=%.2f",
                goal_pose->pose.position.x, goal_pose->pose.position.y);

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    send_goal_options.result_callback =
        [this](const GoalHandleNavigateToPose::WrappedResult & result) {
            switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(this->get_logger(), "Goal reached successfully!");
                    if (current_goal_.first != 0.0 || current_goal_.second != 0.0)
                    {
                        visited_frontiers_.emplace_back(current_goal_);
                        RCLCPP_INFO(this->get_logger(), "Marked frontier at x: %.2f, y: %.2f as visited.",
                                    current_goal_.first, current_goal_.second);
                        current_goal_ = std::make_pair(0.0, 0.0);
                    }
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_WARN(this->get_logger(), "Goal was aborted.");
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_WARN(this->get_logger(), "Goal was canceled.");
                    break;
                default:
                    RCLCPP_WARN(this->get_logger(), "Unknown result code.");
                    break;
            }
            // Reset navigation_in_progress_ to allow the next goal selection
            navigation_in_progress_ = false;
        };

    auto future_goal_handle = action_client_->async_send_goal(goal_msg, send_goal_options);
    navigation_in_progress_ = true;
    current_goal_ = std::make_pair(goal_pose->pose.position.x, goal_pose->pose.position.y);
}

// Function to get the current robot pose in the 'map' frame
geometry_msgs::msg::PoseStamped::SharedPtr ExploreNode::get_current_pose()
{
    geometry_msgs::msg::PoseStamped::SharedPtr current_pose = std::make_shared<geometry_msgs::msg::PoseStamped>();
    try
    {
        auto now = this->get_clock()->now();
        // Wait for the transform to be available with a longer timeout
        if (!tf_buffer_.canTransform("map", "base_link", now, rclcpp::Duration::from_seconds(2.0)))
        {
            RCLCPP_WARN(this->get_logger(), "Transform from 'map' to 'base_link' not available. Retrying...");
            return nullptr;
        }

        geometry_msgs::msg::TransformStamped transform = tf_buffer_.lookupTransform(
            "map",
            "base_link",
            now,
            rclcpp::Duration::from_seconds(1.0)
        );

        current_pose->header = transform.header;
        current_pose->pose.position.x = transform.transform.translation.x;
        current_pose->pose.position.y = transform.transform.translation.y;
        current_pose->pose.position.z = transform.transform.translation.z;
        current_pose->pose.orientation = transform.transform.rotation;

        RCLCPP_DEBUG(this->get_logger(), "Current pose: x=%.2f, y=%.2f", current_pose->pose.position.x, current_pose->pose.position.y);
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(this->get_logger(), "Could not get current pose: %s", ex.what());
        return nullptr;
    }

    return current_pose;
}

// Corrected frontier detection function
std::vector<int> ExploreNode::find_frontiers(const nav_msgs::msg::OccupancyGrid &map_data)
{
    int width = map_data.info.width;
    int height = map_data.info.height;
    const std::vector<int8_t> &data = map_data.data;

    std::vector<int> frontier_indices;

    for (int y = 1; y < height -1; ++y)
    {
        for (int x = 1; x < width -1; ++x)
        {
            int idx = y * width + x;
            int cell = data[idx];

            // Check if cell is free
            if (cell >= 0 && cell <= free_threshold_)
            {
                // Check if any neighbor is unknown
                bool is_frontier = false;
                for (int dy = -1; dy <=1; ++dy)
                {
                    for (int dx = -1; dx <=1; ++dx)
                    {
                        if (dx == 0 && dy == 0)
                            continue;
                        int nidx = (y + dy) * width + (x + dx);
                        int ncell = data[nidx];
                        if (ncell == -1)
                        {
                            is_frontier = true;
                            break;
                        }
                    }
                    if (is_frontier)
                        break;
                }
                if (is_frontier)
                {
                    frontier_indices.push_back(idx);
                }
            }
        }
    }

    RCLCPP_INFO(this->get_logger(), "Frontiers found before filtering: %zu", frontier_indices.size());

    // Now, we can cluster the frontier cells using connected components

    // Create a binary image of frontier cells
    cv::Mat frontier_image = cv::Mat::zeros(height, width, CV_8U);
    for (const auto &idx : frontier_indices)
    {
        int y = idx / width;
        int x = idx % width;
        frontier_image.at<uint8_t>(y, x) = 255;
    }

    // Perform connected components
    cv::Mat labels, stats, centroids;
    int num_components = cv::connectedComponentsWithStats(frontier_image, labels, stats, centroids, 8, CV_32S);

    std::vector<int> filtered_frontier_indices;

    for (int i = 1; i < num_components; ++i)
    {
        int size = stats.at<int>(i, cv::CC_STAT_AREA);
        if (size >= min_frontier_size_)
        {
            for (int y = 0; y < height; ++y)
            {
                for (int x = 0; x < width; ++x)
                {
                    if (labels.at<int>(y, x) == i)
                    {
                        int idx = y * width + x;
                        filtered_frontier_indices.push_back(idx);
                    }
                }
            }
        }
    }

    RCLCPP_INFO(this->get_logger(), "Frontiers found after filtering: %zu", filtered_frontier_indices.size());

    return filtered_frontier_indices;
}

// Function to convert frontier indices to coordinates
std::vector<std::pair<double, double>> ExploreNode::get_frontier_coords(const std::vector<int> &frontier_indices)
{
    std::vector<std::pair<double, double>> frontier_coords;

    if(frontier_indices.empty())
        return frontier_coords;

    int width = latest_map_->info.width;
    double resolution = latest_map_->info.resolution;
    double origin_x = latest_map_->info.origin.position.x;
    double origin_y = latest_map_->info.origin.position.y;

    frontier_coords.reserve(frontier_indices.size());

    for(auto idx : frontier_indices)
    {
        int row = idx / width;
        int col = idx % width;

        double x = (col * resolution) + origin_x + (resolution / 2.0);
        double y = (row * resolution) + origin_y + (resolution / 2.0);

        frontier_coords.emplace_back(std::make_pair(x, y));
    }

    RCLCPP_DEBUG(this->get_logger(), "Converted %zu frontiers to coordinates.", frontier_coords.size());
    return frontier_coords;
}

// Function to publish frontiers as markers for RViz
void ExploreNode::publish_frontiers(const std::vector<std::pair<double, double>> &frontier_coords)
{
    visualization_msgs::msg::MarkerArray marker_array;
    int id = 0;
    for(const auto &coord : frontier_coords)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "frontiers";
        marker.id = id++;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = coord.first;
        marker.pose.position.y = coord.second;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;
        marker.color.a = 1.0;
        marker.color.r = 1.0; // Red color for frontiers
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker_array.markers.push_back(marker);
    }
    frontier_pub_->publish(marker_array);
    RCLCPP_DEBUG(this->get_logger(), "Published %zu frontiers for visualization.", frontier_coords.size());
}

// Function to filter frontiers based on visits and attempts
std::vector<std::pair<double, double>> ExploreNode::filter_frontiers(const std::vector<std::pair<double, double>> &frontier_coords)
{
    std::vector<std::pair<double, double>> filtered_coords;

    for(const auto &coord : frontier_coords)
    {
        if(!is_frontier_visited(coord))
        {
            auto key = round_coord(coord);
            int attempts = frontier_selection_attempts_.count(key) ? frontier_selection_attempts_[key] : 0;
            if(attempts < max_attempts_)
            {
                filtered_coords.push_back(coord);
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Frontier at x: %.2f, y: %.2f reached max attempts and will be excluded.", coord.first, coord.second);
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Frontier at x: %.2f, y: %.2f has been visited and will be excluded.", coord.first, coord.second);
        }
    }

    RCLCPP_DEBUG(this->get_logger(), "Frontiers after filtering: %zu", filtered_coords.size());
    return filtered_coords;
}

// Function to check if a frontier has been visited
bool ExploreNode::is_frontier_visited(const std::pair<double, double> &coord)
{
    for(const auto &visited : visited_frontiers_)
    {
        double distance = std::hypot(coord.first - visited.first, coord.second - visited.second);
        if(distance < frontier_visit_threshold_)
            return true;
    }
    return false;
}

// Function to select the best frontier based on information gain and distance
std::pair<double, double> ExploreNode::select_frontier(const std::vector<std::pair<double, double>> &frontier_coords, const geometry_msgs::msg::PoseStamped &current_pose)
{
    if(frontier_coords.empty())
        return std::make_pair(0.0, 0.0);

    std::vector<double> scores;
    scores.reserve(frontier_coords.size());

    for(const auto &coord : frontier_coords)
    {
        double info_gain = information_gain(coord, info_radius_);
        double distance = std::hypot(coord.first - current_pose.pose.position.x, coord.second - current_pose.pose.position.y);
        double score = info_gain / (distance + 1e-6); // Avoid division by zero
        scores.push_back(score);
    }

    // Find the index with the maximum score
    auto max_it = std::max_element(scores.begin(), scores.end());
    if(max_it == scores.end())
        return std::make_pair(0.0, 0.0);

    size_t max_idx = std::distance(scores.begin(), max_it);
    auto selected_coord = frontier_coords[max_idx];

    // Increment selection attempts
    auto key = round_coord(selected_coord);
    frontier_selection_attempts_[key] += 1;

    RCLCPP_INFO(this->get_logger(), "Selected frontier at x: %.2f, y: %.2f with score %.2f. Attempts: %d",
                selected_coord.first, selected_coord.second, scores[max_idx], frontier_selection_attempts_[key]);

    return selected_coord;
}

// Function to calculate information gain
double ExploreNode::information_gain(const std::pair<double, double> &coord, double radius)
{
    const auto &map_data = *latest_map_;
    int width = map_data.info.width;
    int height = map_data.info.height;
    double resolution = map_data.info.resolution;
    double origin_x = map_data.info.origin.position.x;
    double origin_y = map_data.info.origin.position.y;

    // Convert coordinates to grid indices
    int center_x = static_cast<int>((coord.first - origin_x) / resolution);
    int center_y = static_cast<int>((coord.second - origin_y) / resolution);
    int radius_cells = static_cast<int>(radius / resolution);

    // Define window boundaries
    int x_min = std::max(0, center_x - radius_cells);
    int x_max = std::min(width, center_x + radius_cells);
    int y_min = std::max(0, center_y - radius_cells);
    int y_max = std::min(height, center_y + radius_cells);

    // Count unknown cells in the radius
    int unknown_cells = 0;
    for(int y = y_min; y < y_max; ++y)
    {
        for(int x = x_min; x < x_max; ++x)
        {
            int idx = y * width + x;
            if(map_data.data[idx] == -1)
                unknown_cells++;
        }
    }

    double info_gain = unknown_cells * (resolution * resolution);

    RCLCPP_DEBUG(this->get_logger(), "Information gain for frontier at x: %.2f, y: %.2f is %.2f", coord.first, coord.second, info_gain);
    return info_gain;
}

// Function to validate the goal position
bool ExploreNode::is_goal_position_valid(const geometry_msgs::msg::PoseStamped &goal_pose)
{
    const auto &map_data = *latest_map_;
    int width = map_data.info.width;
    int height = map_data.info.height;
    double resolution = map_data.info.resolution;
    double origin_x = map_data.info.origin.position.x;
    double origin_y = map_data.info.origin.position.y;

    // Convert coordinates to grid indices
    int col = static_cast<int>((goal_pose.pose.position.x - origin_x) / resolution);
    int row = static_cast<int>((goal_pose.pose.position.y - origin_y) / resolution);

    // Check boundaries
    if(col < 0 || col >= width || row < 0 || row >= height)
    {
        RCLCPP_WARN(this->get_logger(), "Goal position (%.2f, %.2f) is out of map bounds.",
                    goal_pose.pose.position.x, goal_pose.pose.position.y);
        return false;
    }

    int index = row * width + col;
    int cell_value = map_data.data[index];

    if(cell_value < 0 || cell_value > free_threshold_)
    {
        RCLCPP_WARN(this->get_logger(), "Goal position (%.2f, %.2f) is not free (cell value: %d).",
                    goal_pose.pose.position.x, goal_pose.pose.position.y, cell_value);
        return false;
    }

    return true;
}

// Function to mark a frontier as invalid
void ExploreNode::mark_frontier_as_invalid(const std::pair<double, double> &coord)
{
    auto key = round_coord(coord);
    frontier_selection_attempts_[key] += 1;
    RCLCPP_INFO(this->get_logger(), "Marked frontier at x: %.2f, y: %.2f as invalid. Attempts: %d",
                coord.first, coord.second, frontier_selection_attempts_[key]);
}

// Function to round coordinates to a specific precision and convert to a key
std::pair<int, int> ExploreNode::round_coord(const std::pair<double, double> &coord, int precision)
{
    int x = static_cast<int>(std::round(coord.first * std::pow(10, precision)));
    int y = static_cast<int>(std::round(coord.second * std::pow(10, precision)));
    return std::make_pair(x, y);
}

// Main function
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ExploreNode>();

    try
    {
        rclcpp::spin(node);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}
