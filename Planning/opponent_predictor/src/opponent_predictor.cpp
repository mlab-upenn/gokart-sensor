#include <fstream>
#include <cmath>

#include "opponent_predictor.h"

OpponentPredictor::OpponentPredictor() : Node("opponent_predictor_node") {
    // ROS Params
    this->declare_parameter("visualize");
    this->declare_parameter("visualize_grid");
    this->declare_parameter("visualize_obstacle");
    this->declare_parameter("visualize_opp");
    this->declare_parameter("visualize_opp_pose");
    this->declare_parameter("visualize_opp_bbox");

    this->declare_parameter("grid_xmin");
    this->declare_parameter("grid_xmax");
    this->declare_parameter("grid_ymin");
    this->declare_parameter("grid_ymax");
    this->declare_parameter("grid_resolution");
    this->declare_parameter("plot_resolution");
    this->declare_parameter("grid_safe_dist");
    this->declare_parameter("goal_safe_dist");

    this->declare_parameter("cluster_dist_tol");
    this->declare_parameter("cluster_size_tol");

    // General Variables
    pose_topic = "/gnss_ekf";
    scan_topic = "/ouster/scan";

    // Timers
    timer_ = this->create_wall_timer(1s, std::bind(&OpponentPredictor::timer_callback, this));

    // Subscribers
    pose_sub_ = this->create_subscription<Odometry>(pose_topic, 1, std::bind(&OpponentPredictor::pose_callback, this,
                                                                             std::placeholders::_1));
    scan_sub_ = this->create_subscription<LaserScan>(scan_topic, 1, std::bind(&OpponentPredictor::scan_callback, this,
                                                                              std::placeholders::_1));

    // Publishers
    fps_pub_ = this->create_publisher<Int16>(fps_topic, 10);

    opp_state_pub_ = this->create_publisher<PoseStamped>(opp_state_topic, 10);
    opp_bbox_pub_ = this->create_publisher<PoseArray>(opp_bbox_topic, 10);

    grid_pub_ = this->create_publisher<MarkerArray>(grid_topic, 10);
    obstacle_pub_ = this->create_publisher<MarkerArray>(obstacle_topic, 10);
    opp_viz_pose_pub_ = this->create_publisher<Marker>(opp_viz_pose_topic, 10);
    opp_viz_bbox_pub_ = this->create_publisher<MarkerArray>(opp_viz_bbox_topic, 10);
}

void OpponentPredictor::timer_callback() {
    Int16 fps;
    fps.data = frame_cnt;
    frame_cnt = 0;
    fps_pub_->publish(fps);
    RCLCPP_INFO(this->get_logger(), "fps: %d", fps.data);
}

void OpponentPredictor::scan_callback(const LaserScan::ConstSharedPtr scan_msg) {
    vector<float> ranges(scan_msg->ranges);
    for (auto &range: ranges) {
        if (range < scan_msg->range_min) {
            range = scan_msg->range_min;
        } else if (range > scan_msg->range_max) {
            range = scan_msg->range_max;
        }
    }

    double xmin = this->get_parameter("grid_xmin").as_double();
    double xmax = this->get_parameter("grid_xmax").as_double();
    double ymin = this->get_parameter("grid_ymin").as_double();
    double ymax = this->get_parameter("grid_ymax").as_double();
    double resolution = this->get_parameter("grid_resolution").as_double();
    double grid_safe_dist = this->get_parameter("grid_safe_dist").as_double();

    int nx = int((xmax - xmin) / resolution) + 1;
    int ny = int((ymax - ymin) / resolution) + 1;

    double x_resolution = (xmax - xmin) / (nx - 1);
    double y_resolution = (ymax - ymin) / (ny - 1);

    // Discretize x and y
    vector<double> xs(nx), ys(ny);
    vector<double>::iterator ptr;
    double val;
    for (ptr = xs.begin(), val = xmin; ptr != xs.end(); ++ptr) {
        *ptr = val;
        val += x_resolution;
    }
    for (ptr = ys.begin(), val = ymin; ptr != ys.end(); ++ptr) {
        *ptr = val;
        val += y_resolution;
    }

    if (grid.empty()) {
        vector<vector<double>> grid_v(nx, vector<double>(ny, -1e8));
        vector<vector<double>> grid_x(nx, vector<double>(ny, -1e8));
        vector<vector<double>> grid_y(nx, vector<double>(ny, -1e8));

        grid.push_back(grid_v);
        grid.push_back(grid_x);
        grid.push_back(grid_y);
    }

    for (int i = 0; i < nx; ++i) {
        double x = xs[i];
        for (int j = 0; j < ny; ++j) {
            double y = ys[j];
            double rho = sqrt(x * x + y * y);
            double phi = atan2(y, x);
            int ray_idx = int((phi - scan_msg->angle_min) / scan_msg->angle_increment);

            grid[0][i][j] = (abs(rho - ranges[ray_idx]) < grid_safe_dist);
            grid[1][i][j] = x;
            grid[2][i][j] = y;
        }
    }

    // Record current time stamp
    scan_sec = scan_msg->header.stamp.sec;
    scan_nanosec = scan_msg->header.stamp.nanosec;
}

void OpponentPredictor::pose_callback(const Odometry::ConstSharedPtr pose_msg) {
    double curr_x = pose_msg->pose.pose.position.x;
    double curr_y = pose_msg->pose.pose.position.y;
    double quat_x = pose_msg->pose.pose.orientation.x;
    double quat_y = pose_msg->pose.pose.orientation.y;
    double quat_z = pose_msg->pose.pose.orientation.z;
    double quat_w = pose_msg->pose.pose.orientation.w;

    ego_global_pos = {curr_x, curr_y};
    ego_global_yaw = atan2(2 * (quat_w * quat_z + quat_x * quat_y),
                           1 - 2 * (quat_y * quat_y + quat_z * quat_z));

    // Find opponent
    get_opponent();

    // Visualization
    bool visualize = this->get_parameter("visualize").as_bool();
    bool visualize_grid = this->get_parameter("visualize_grid").as_bool();
    bool visualize_obs = this->get_parameter("visualize_obstacle").as_bool();
    bool visualize_opp_pose = this->get_parameter("visualize_opp_pose").as_bool();
    bool visualize_opp_bbox = this->get_parameter("visualize_opp_bbox").as_bool();

    if (visualize) {
        if (visualize_grid) {
            visualize_occupancy_grid();
        }
        if (visualize_obs) {
            visualize_obstacle();
        }
        if (visualize_opp_pose) {
            visualize_opponent_pose();
        }
        if (visualize_opp_bbox) {
            visualize_opponent_bbox();
        }
    }

    // Increase frame count
    frame_cnt++;
}


void OpponentPredictor::get_opponent() {
    // If laser scan not available yet, return
    if (grid.empty()) {
        return;
    }

    // Clear containers
    opp_global_pos.clear();
    obstacle.clear();
    opponent.clear();

    // Calculate grid points in map frame
    vector<vector<double>> grid_point;

    int nx = (int) grid[0].size();
    int ny = (int) grid[0][0].size();
    for (int i = 0; i < nx; ++i) {
        for (int j = 0; j < ny; ++j) {
            if (grid[0][i][j] == 0.0) continue;
            double local_x = grid[1][i][j];
            double local_y = grid[2][i][j];

            double global_x = cos(ego_global_yaw) * local_x - sin(ego_global_yaw) * local_y + ego_global_pos[0];
            double global_y = sin(ego_global_yaw) * local_x + cos(ego_global_yaw) * local_y + ego_global_pos[1];

            grid_point.push_back({global_x, global_y});
        }
    }

    // Find the largest cluster and large enough as opponent car
    for (int i=0; i<(int)grid_point.size(); i++) {
        obstacle.push_back(grid_point[i]);
    }

    double cluster_dist_tol = this->get_parameter("cluster_dist_tol").as_double();
    int cluster_size_tol = (int) this->get_parameter("cluster_size_tol").as_int();
    vector<vector<int>> clusters = cluster(obstacle, cluster_dist_tol);
    int max_cluster_size = 0;
    int max_cluster_idx = 0;
    for (int i = 0; i < (int) clusters.size(); ++i) {
        if ((int) clusters[i].size() > max_cluster_size) {
            max_cluster_size = (int) clusters[i].size();
            max_cluster_idx = i;
        }
    }

    if (max_cluster_size < cluster_size_tol) {
        PoseStamped opp_state;
        opp_state.pose.position.x = INFINITY;
        opp_state.pose.position.y = INFINITY;
        opp_state.header.stamp.sec = scan_sec;
        opp_state.header.stamp.nanosec = scan_nanosec;
        opp_state_pub_->publish(opp_state);
        return;
    }

    vector<int> opponent_idx = clusters[max_cluster_idx];
    for (const auto i: opponent_idx) {
        opponent.push_back(obstacle[i]);
    }

    // Use average pose for estimation
    double mean_x = 0.0, mean_y = 0.0;
    for (const auto &pt: opponent) {
        mean_x += pt[0];
        mean_y += pt[1];
    }
    mean_x /= double(opponent.size());
    mean_y /= double(opponent.size());
    cout << "opponent_size" << ": " << opponent.size() << endl;
    opp_global_pos = {mean_x, mean_y};

    // Publish result
    PoseStamped opp_state;
    opp_state.pose.position.x = opp_global_pos[0];
    opp_state.pose.position.y = opp_global_pos[1];
    opp_state.header.stamp.sec = scan_sec;
    opp_state.header.stamp.nanosec = scan_nanosec;
    opp_state_pub_->publish(opp_state);
    // cout << opp_global_pos[0] << ' ' << opp_global_pos[1] << endl;

    PoseArray opp_bbox;
    opp_bbox.header.frame_id = "/map";
    opp_bbox.header.stamp.sec = scan_sec;
    opp_bbox.header.stamp.nanosec = scan_nanosec;
    for (const auto &pt: opponent) {
        Pose pose;
        pose.position.x = pt[0];
        pose.position.y = pt[1];
        opp_bbox.poses.push_back(pose);
    }
    opp_bbox_pub_->publish(opp_bbox);
}
