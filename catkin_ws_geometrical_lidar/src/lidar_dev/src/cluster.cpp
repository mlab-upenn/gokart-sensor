// process the /ground_segmentation/obstacle_cloud output from the segmentation node

#include "cluster.h"

ros::Publisher pub;
ros::Publisher markers_pub; // publisher for cylinder markers

ClusterParams params;
using namespace std;

// compute the number of expected points for cone object
int num_expected_points(const pcl::PointXYZ &centre) {
    double d = sqrt(centre.x * centre.x + centre.y * centre.y + centre.z * centre.z);
    static double hc = 0.70;               // cone height
    static double wc = 0.27;               // cone width
    static double rv = 0.00698;  // angular resolution vertical
    static double rh = 0.00349;    // angular resolution horizontal

    // compute and return number of expected points
    double E = 0.5 * hc / (2 * d * tan(rv / 2)) * wc / (2 * d * tan(rh / 2));
    return (int)E;
}


 std::vector<float> extract_column(const std::vector<std::vector<float>>& matrix, int col_idx) {
      std::vector<float> result;
      for(const auto& row : matrix) {
          result.push_back(row[col_idx]);
      }
      return result;
 }


// function to set the marker properties
void set_marker_properties(
    visualization_msgs::Marker *marker, 
    pcl::PointXYZ centre, 
    int n, 
    std::string frame_id, double confidence)
{
    marker->header.frame_id = frame_id;
    marker->header.stamp = ros::Time();
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
    // marker->color.r = params.marker_r;
    marker->color.r = 0.5;
    marker->color.g = 1.0;
    marker->color.b = params.marker_b;

    marker->lifetime = ros::Duration(0.1);
}

// perform euclidean clustering
void cloud_cluster_cb(const sensor_msgs::PointCloud2ConstPtr &obstacles_msg, const sensor_msgs::PointCloud2ConstPtr &ground_msg)
{
    // time callback run time as performance measure
    ros::WallTime start_, end_;
    start_ = ros::WallTime::now();
    
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
    pcl::PointCloud<pcl::PointXYZ> testcloud = *cloud_filtered;
    // vector to store marker points
    std::vector<pcl::PointXYZ> marker_points;
    std::vector<double> conf;


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

        // skip processing step if there is insufficient points
        int expected = num_expected_points(centre);
        if (cloud_cluster->size() < 0.60 * expected) {
            continue;
        }


        
        pcl::PointCloud<pcl::PointXYZ> testcloud2 = *cloud_cluster;
        std::vector<std::vector<double>> cpoints;
        std::vector<double> zvals;
        std::vector<double> xvals;
        std::vector<double> yvals;
        // std::vector<double> conf;
        int perc_75_100 = 0;
        int perc_50_75 = 0;
        int perc_25_50 = 0;
        int perc_0_25 = 0;

        for (auto i: testcloud2.points)
        {
            zvals.push_back(i.z);
            xvals.push_back(i.x);
            yvals.push_back(i.y);

        }

        double min_zvals = *std::min_element(zvals.begin(), zvals.end());
        double max_zvals = *std::max_element(zvals.begin(), zvals.end());
        double avg_z = max_zvals - min_zvals; 

        for (auto i: testcloud2.points)
        {
            // std::cout << i << ' ' << std::endl;
            double c_z = i.z;
            if(c_z > max_zvals - 0.25 * avg_z)
            {
                perc_75_100 += 1;
            }
            else if (c_z <= max_zvals - 0.25 * avg_z && c_z > max_zvals - 0.5 * avg_z)
            {
                perc_50_75 += 1;
            }
            else if (c_z <= max_zvals - 0.5 * avg_z && c_z > max_zvals - 0.75 * avg_z)
            {
                perc_25_50 += 1;
            }
            else
            {
                perc_0_25 += 1;
            }

        }
        std::cout << "X: " << centre.x << ",Y: " << centre.y << ",Z: " << centre.z << std::endl;;
        std::cout << "Points in top 25:" << perc_75_100 << std::endl;
        std::cout << "Points in bottom 25:" << perc_0_25 << std::endl;

        cout << "Highest z value" << max_zvals << endl;
        cout << "Lowest z value" << min_zvals << endl;
        cout << avg_z << endl;

        // old if clause if(perc_75_100 > perc_25_50 || perc_75_100 > perc_0_25 || perc_50_75 > perc_0_25)
        if(perc_75_100 > perc_0_25)
        {
            // std::cout << "cone upper points faulty SKIPPING by " << upcen_p - lowcen_p << std::endl;
            std::cout << "cone upper points faulty SKIPPING  " << (perc_75_100 > perc_25_50) << (perc_75_100 > perc_0_25) << (perc_50_75 > perc_0_25) << endl;
            // continue;
        }
        std::cout << "[confirmed] num_expected_points = " << 0.60 * num_expected_points(centre) << std::endl;
        std::cout << "[confirmed] num actual points   = " << cloud_cluster->size() << std::endl;
        std::cout << "[confirmed] distance to cone    = " << d << std::endl;

        int val = 0; //lowcen_p/upcen_p;
        if(val>1) val = 1;
        conf.push_back(val);
        cout<< "val" <<endl;
        cout<< val <<endl;

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
        set_marker_properties(&marker_array_msg.markers[i], marker_points[i], i, ground->header.frame_id, conf[i]);
    }

    // std::cout << "NUM OF MARKERS = " << marker_points.size() << std::endl;
    

    // set additional header info and convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*clustered_cloud, output);
    output.header.frame_id = ground->header.frame_id;
    output.header.stamp = ros::Time::now();

    // ROS_INFO("About to publish cluster output \n");

    // publish the output data
    pub.publish(output);
    markers_pub.publish(marker_array_msg);

    // measure and print runtime performance
    end_ = ros::WallTime::now();
    double execution_time = (end_ - start_).toNSec() * 1e-6;
    ROS_INFO_STREAM("Exectution time (ms): " << execution_time);
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
