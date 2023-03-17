// process the /ground_segmentation/obstacle_cloud output from the segmentation node

#include "cluster.h"

ros::Publisher pub;
ros::Publisher markers_pub; // publisher for cylinder markers

ClusterParams params;

int counter = 0;
double avg_inference_time = 0;

// compute the number of expected points for cone object
int num_expected_points(const pcl::PointXYZ &centre) {
    double d = sqrt(centre.x * centre.x + centre.y * centre.y + centre.z * centre.z);
    static double hc = 0.7;               // cone height
    static double wc = 0.27;               // cone width
    static double rv = 0.00698;  // angular resolution vertical
    static double rh = 0.00349;    // angular resolution horizontal

    // compute and return number of expected points
    double E = 0.5 * hc / (2 * d * tan(rv / 2)) * wc / (2 * d * tan(rh / 2));
    return (int)E;
}

// perform euclidean clustering
void cloud_cluster_cb(const sensor_msgs::PointCloud2ConstPtr &obstacles_msg, const sensor_msgs::PointCloud2ConstPtr &ground_msg)
{
    // time callback run time as performance measure
    ros::WallTime start_, end_;
    start_ = ros::WallTime::now();

    counter += 1;

    double time_header_sec = obstacles_msg->header.stamp.sec;
    double time_header_nsec = obstacles_msg->header.stamp.nsec;
    std::cout << "---------------------------------" << time_header_sec << std::endl;
    
    // container for ground data
    pcl::PCLPointCloud2 *ground = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr groundPtr(ground);
    
    // container for original & filtered data
    pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);

    // convert given message into PCL data type
    pcl_conversions::toPCL(*obstacles_msg, *cloud);
    pcl_conversions::toPCL(*ground_msg, *ground);

    // convert from PointCloud2 to PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr input(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_ground (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*cloud, *input);
    pcl::fromPCLPointCloud2(*ground, *input_ground);

    ROS_INFO("There are %ld points in input point cloud \n", input->size());

    // Use a conditional filter to remove points at the origin (0, 0, 0)
    pcl::ConditionOr<pcl::PointXYZ>::Ptr range_cond(new pcl::ConditionOr<pcl::PointXYZ>());
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::GT, 0.0)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::GT, 0.0)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, 0.0)));

    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::LT, 0.0)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::LT, 0.0)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, 0.0)));

    // build the filter
    pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
    condrem.setCondition(range_cond);
    condrem.setInputCloud(input);
    condrem.setKeepOrganized(false);

    // apply filter
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    condrem.filter(*cloud_filtered);
    std::cout << "PointCloud after conditional filtering has: " << cloud_filtered->points.size() << " data points." << std::endl;

    // create KdTree object
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(params.cluster_tol);     // 8cm (affects resulting cluster size)
    ec.setMinClusterSize(params.cluster_min);       // minimum number of points
    ec.setMaxClusterSize(params.cluster_max);       // maximum number of points
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);
    ec.extract(cluster_indices);

    pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // vector to store marker points
    std::vector<pcl::PointXYZ> marker_points;

    // outer loop goes through all the clusters we found
    for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);

        for (const int &index : it->indices)
            cloud_cluster->points.push_back(cloud_filtered->points[index]);

        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        // extract centroid of cluster
        pcl::PointXYZ centre;
        pcl::computeCentroid(*cloud_cluster, centre);

        double d = sqrt(centre.x * centre.x + centre.y * centre.y + centre.z * centre.z);
        std::cout << "[potential] num_expected_points = " << 0.60 * num_expected_points(centre) << std::endl;
        std::cout << "[potential] num actual points   = " << cloud_cluster->size() << std::endl;
        std::cout << "[potential] distance to cone    = " << d << std::endl;

        // skip processing step if there is insufficient points
        int expected = num_expected_points(centre);
        if (cloud_cluster->size() < 0.60 * expected) {
            continue;
        }

        std::cout << "[confirmed] num_expected_points = " << 0.60 * num_expected_points(centre) << std::endl;
        std::cout << "[confirmed] num actual points   = " << cloud_cluster->size() << std::endl;
        std::cout << "[confirmed] distance to cone    = " << d << std::endl;

        // add to marker points
        marker_points.push_back(centre);

        // cylindrical reconstruction from ground points
        pcl::ConditionAnd<pcl::PointXYZ>::Ptr cyl_cond (new pcl::ConditionAnd<pcl::PointXYZ> ());

        Eigen::Matrix3f cylinderMatrix;
        cylinderMatrix(0,0) = 1.0;
        cylinderMatrix(1,1) = 1.0;

        Eigen::Vector3f cylinderPosition;
        cylinderPosition << -centre.x, -centre.y, 0;

        double radius = params.reconst_radius;
        float cylinderScalar = -(radius * radius) + centre.x * centre.x + centre.y * centre.y;

        pcl::TfQuadraticXYZComparison<pcl::PointXYZ>::Ptr cyl_comp 
            (new pcl::TfQuadraticXYZComparison<pcl::PointXYZ> 
            (pcl::ComparisonOps::LE, cylinderMatrix, cylinderPosition, cylinderScalar));
        cyl_cond->addComparison(cyl_comp);

        pcl::PointCloud<pcl::PointXYZ> recovered;

        // build and apply filter
        condrem.setCondition(cyl_cond);
        condrem.setInputCloud(input_ground);
        condrem.setKeepOrganized(false);
        condrem.filter(recovered);

        // join each cloud cluster into one combined cluster (visualisation)
        *clustered_cloud += *cloud_cluster + recovered;

        // print info about cluster size
        // std::cout << n << " PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
    }

    // prepare marker array
    visualization_msgs::MarkerArray marker_array_msg;
    marker_array_msg.markers.resize(marker_points.size());
    for (int i = 0; i < marker_points.size(); ++i) {
        set_marker_properties(&marker_array_msg.markers[i], marker_points[i], i, ground->header.frame_id, time_header_sec, time_header_sec);
    }

    std::cout << "NUM OF MARKERS = " << marker_points.size() << std::endl;
    

    // set additional header info and convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*clustered_cloud, output);
    output.header.frame_id = ground->header.frame_id;
    output.header.stamp = ros::Time::now();

    ROS_INFO("About to publish cluster output \n");

    // publish the output data
    pub.publish(output);
    markers_pub.publish(marker_array_msg);

    // measure and print runtime performance
    end_ = ros::WallTime::now();
    double execution_time = (end_ - start_).toNSec() * 1e-6;
    avg_inference_time += execution_time;
    ROS_INFO_STREAM("Exectution time (ms): " << execution_time);
    ROS_INFO_STREAM("Average Exectution time (ms): " << avg_inference_time / counter);
}

// function to set the marker properties
void set_marker_properties(
    visualization_msgs::Marker *marker, 
    pcl::PointXYZ centre, 
    int n, 
    std::string frame_id,
    double time_head_sec,
    double time_head_nsec)
{
    marker->header.frame_id = frame_id;
    marker->header.stamp.sec = time_head_sec;
    marker->header.stamp.nsec = time_head_nsec;
    marker->ns = "my_namespace";
    marker->id = n;
    marker->type = visualization_msgs::Marker::CYLINDER;
    marker->action = visualization_msgs::Marker::ADD;

    marker->pose.position.x = centre.x;
    marker->pose.position.y = centre.y;
    marker->pose.position.z = centre.z;

    marker->pose.orientation.x = 0.0;
    marker->pose.orientation.y = 0.0;
    marker->pose.orientation.z = 0.0;
    marker->pose.orientation.w = 1.0;

    marker->scale.x = params.marker_sx;
    marker->scale.y = params.marker_sy;
    marker->scale.z = params.marker_sz;

    // alpha and RGB settings
    marker->color.a = params.marker_alpha;
    marker->color.r = params.marker_r;
    marker->color.g = params.marker_g;
    marker->color.b = params.marker_b;

    marker->lifetime = ros::Duration(0.1);
}

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "pcl_boxcrop");
    ros::NodeHandle nh;

    // Parse parameters
    nh.param("/cluster/cluster_tol", params.cluster_tol, params.cluster_tol);
    nh.param("/cluster/cluster_min", params.cluster_min, params.cluster_min);
    nh.param("/cluster/cluster_max", params.cluster_max, params.cluster_max);
    nh.param("/cluster/reconst_radius", params.reconst_radius, params.reconst_radius);
    nh.param("/cluster/marker_sx", params.marker_sx, params.marker_sx);
    nh.param("/cluster/marker_sy", params.marker_sy, params.marker_sy);
    nh.param("/cluster/marker_sz", params.marker_sz, params.marker_sz);
    nh.param("/cluster/marker_alpha", params.marker_alpha, params.marker_alpha);
    nh.param("/cluster/marker_r", params.marker_r, params.marker_r);
    nh.param("/cluster/marker_g", params.marker_g, params.marker_g);
    nh.param("/cluster/marker_b", params.marker_b, params.marker_b);

    // Create a ROS subscriber for ground plane and potential obstacles
    message_filters::Subscriber<sensor_msgs::PointCloud2> ground_sub(nh, "ground_segmentation/obstacle_cloud", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> obstacles_sub(nh, "ground_segmentation/ground_cloud", 1);

    // Pass both subscribed message into the same callback
    message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> sync(ground_sub, obstacles_sub, 10);
    sync.registerCallback(boost::bind(&cloud_cluster_cb, _1, _2));

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2>("cluster_output", 1);
    markers_pub = nh.advertise<visualization_msgs::MarkerArray>("cluster_markers", 1);

    // Spin
    ros::spin();
}