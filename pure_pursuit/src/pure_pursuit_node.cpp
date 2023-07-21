#include <sstream>
#include <string>
#include <cmath>
#include <vector>
#include <iostream>
#include <fstream>
#include <Eigen/Geometry>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"


using std::placeholders::_1;
using namespace std;

class PurePursuit : public rclcpp::Node
{
private:
    string path = "/home/jet/sim_ws/src/lab6-slam-and-pure-pursuit-team_02-main/pure_pursuit/checkpoints/";
    string filename = "waypoints.csv";
    vector<vector<double>> waypoints;

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vis_goal_pub ;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vis_path_pub ;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::ConstSharedPtr odom_sub;

    // hyperparameters here
    double look_ahead, kp, vel_min, vel_max, vel_inter, current_vel;

    void set_hyperparamets()
    {
        rclcpp::Parameter look_ahead_param = this->get_parameter("look_ahead");
        rclcpp::Parameter kp_param = this->get_parameter("kp");
        rclcpp::Parameter vel_min_param = this->get_parameter("vel_min");
        rclcpp::Parameter vel_max_param = this->get_parameter("vel_max");
        rclcpp::Parameter vel_inter_param = this->get_parameter("vel_inter");

        look_ahead = look_ahead_param.as_double();
        kp = kp_param.as_double();
        vel_min = vel_min_param.as_double();
        vel_max = vel_max_param.as_double();
        vel_inter = vel_inter_param.as_double();

    }

public:
    PurePursuit() : Node("pure_pursuit_node")
    {
        
        // pf_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&WallFollow::scan_callback, this, _1));
        this->declare_parameter("look_ahead", 1.25);
        this->declare_parameter("kp", 0.55);
        this->declare_parameter("vel_min", 3.0);
        this->declare_parameter("vel_max", 6.0);
        this->declare_parameter("vel_inter", 4.0);

        //simulation
        drive_pub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);
        vis_goal_pub = this->create_publisher<visualization_msgs::msg::Marker>( "visualization_goal_topic", 10 );
        vis_path_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>( "visualization_path_topic", 10);
        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/ego_racecar/odom", 10, std::bind(&PurePursuit::pose_callback, this, _1));
        
        
        //real car
        odom_sub = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>("/drive_info_from_nucleo", 10, std::bind(&PurePursuit::odom_callback, this, _1));
        drive_pub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/autonomous_command_to_nucleo", 10);
        pose_sub = this->create_publisher<geometry_msgs::msg::PoseWithCovariancestamped>("/gnss_ekf", 10,std::bind(&PurePursuit::pose_callback, this, _1);
        parseCSV();
    }

    double get_vel(double x)
    {
        double steer_max = 0.5;
        double a = (vel_min - vel_max) / (steer_max*steer_max);
        return a * (x*x) + vel_max;
    }
    void odom_callback(const ackermann_msgs::msg::AckermannDriveStamped odom_msg){
        current_vel = odom_msg.drive.speed;
    }

    void pose_callback(const geometry::msg::PoseWithCovarianceStamped::ConstSharedPtr odom_msg) // use this for sim, odom
    { 
      
        set_hyperparamets();
        geometry_msgs::msg::Pose::ConstSharedPtr pose_msg = std::make_shared<geometry_msgs::msg::Pose>(odom_msg->pose.pose); // use this only for sim
        
        tf2::Quaternion q(pose_msg->orientation.x, pose_msg->orientation.y, pose_msg->orientation.z, pose_msg->orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch ,yaw;
        m.getRPY(roll,pitch,yaw);

        double base_x = pose_msg->position.x;
        double base_y = pose_msg->position.y;
        double min_dist = 100000;
        double min_y = 0;
        double velocity = 0;
        int min_idx = 0;
        
        for(int i=0; i<waypoints.size(); ++i)
        {
            //always checks in robot origin frame
            double x_local = (waypoints[i][0]-base_x) * cos(yaw) + (waypoints[i][1]-base_y) * sin(yaw);
            double y_local = -(waypoints[i][0]-base_x) * sin(yaw) + (waypoints[i][1]-base_y) * cos(yaw);
            double dist = sqrt(x_local*x_local + y_local*y_local);
            double theta = atan2(y_local, x_local);
            if(  (dist >= look_ahead && dist < min_dist) && (-M_PI/4 < theta && theta < M_PI/4)  )
            {
                min_dist = dist;
                min_idx = i;
                min_y = y_local;
            }            
        }
        velocity = 4.0;

        //turn radius from pure pursuit logic
        double gamma = 2 * min_y / (min_dist*min_dist); 


        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.drive.steering_angle = kp * gamma;
        drive_msg.drive.speed = velocity;
        drive_pub->publish(drive_msg);

        RCLCPP_INFO(this->get_logger(), "VeL: %f", drive_msg.drive.speed);


        // visualize goal marker
        auto marker = visualization_msgs::msg::Marker();
        marker.header = odom_msg->header;
        marker.header.frame_id = "map";
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.15;
        marker.scale.y = 0.15;
        marker.scale.z = 0.15;
        marker.color.a = 1.0; 
        marker.color.r = 1.0;
        marker.pose.position.x = waypoints[min_idx][0];
        marker.pose.position.y = waypoints[min_idx][1];
        marker.id += 1;
        vis_goal_pub->publish(marker);
    }

    void visualize_path_points()
    {
        auto marker_array = visualization_msgs::msg::MarkerArray();
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "map";
        marker.header.stamp = rclcpp::Clock().now();
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.15;
        marker.scale.y = 0.15;
        marker.scale.z = 0.15;
        marker.color.a = 1.0; 
        marker.color.g = 1.0;

        for(int i=0; i<waypoints.size(); ++i)
        {
            marker.pose.position.x = waypoints[i][0];
            marker.pose.position.y = waypoints[i][1];
            marker.id = i;
            marker_array.markers.push_back(marker);
        }

        vis_path_pub->publish(marker_array);
    }

    void parseCSV()
    {
        vector<double> row;
        string line, word;
     
        fstream file(path+filename, ios::in);
        if(file.is_open())
        {
            while(getline(file, line))
            {
                row.clear();
     
                stringstream str(line);
     
                while(getline(str, word, ','))
                    row.push_back(stod(word)); // string to double
                waypoints.push_back(row);
            }

            visualize_path_points();
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "\n\n!!!File could not be opened!!!\n");
        }
    }

    ~PurePursuit() {}
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuit>());
    rclcpp::shutdown();
    return 0;
}