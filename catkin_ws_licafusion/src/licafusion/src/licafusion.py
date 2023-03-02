import rospy
from sensor_msgs.msg import PointCloud2 as pc2
from sensor_msgs.msg import PointField
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import message_filters
import numpy as np
import ros_numpy
from darknet_ros_msgs.msg import BoundingBoxes
from cv_bridge import CvBridge
from sensor_msgs import point_cloud2
from scipy.spatial import distance
# visualization marker ROS
from visualization_msgs.msg import MarkerArray


# LIBRARIES lidar_rings

rospy.init_node('licafusion', anonymous=True)
pub = rospy.Publisher("/lidar_0/m1600/pcl2_XYZIR", pc2, queue_size=10)
bridge = CvBridge()
def project_points(projection_matrix, points):
    """
    projection_matrix: The projection matrix based on the intrinsic camera calibration.
    points: Nx[x,y,z,1.0] np.array of the points that need to be projected to the camera image.
    return: Nx[u,v] rounded coordinates of the points in the camera image as np.int data type.
    """
    assert points.shape[-1] == 4
    points = points[points[:,2] > 0.0]
    uvs = np.matmul(projection_matrix,points.T)
    uvs = uvs/uvs[2,:]
    
    uvs = uvs[0:2,:]
    uvs = uvs.astype(np.int)

    return uvs.T

def callback(image, pcl, bb):
    xyz_arr = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pcl)
    point_arr = np.ones((xyz_arr.shape[0]))
    h_array = np.column_stack((xyz_arr, point_arr))

    c_t_l = np.array([[-0.00423105 , -0.99987 ,-0.0155724,0.09296],
                    [0.0117262,0.0155218,-0.999811,-0.0837121],
                    [0.999922,-0.00441285,0.011659,-0.0289284],
                    [0,0,0,1]])
    proj_mat = np.array([[1442.193090,0.0,943.936435,0.0],
                    [0.0,1440.905704,542.578706,0.0],
                    [ 0.0,0.0,1.0,0.0]])
    cam_points = c_t_l@h_array.T
    uvs = project_points(proj_mat, cam_points.T)

    point_in_fovx = np.bitwise_and([uvs[:,0] >= 0], [(uvs[:,0]) < image.width])
    point_in_fovy = np.bitwise_and([uvs[:,1] >= 0], [(uvs[:,1]) < image.height])
    point_in_fov = np.bitwise_and(point_in_fovx, point_in_fovy)
    uvs = uvs[np.squeeze(point_in_fov),:]
    depth = h_array[np.squeeze(point_in_fov),:]


    fields = [PointField('x', 0, PointField.FLOAT32, 1),
          PointField('y', 4, PointField.FLOAT32, 1),
          PointField('z', 8, PointField.FLOAT32, 1)
          ]

    bb_arr = bb.bounding_boxes
    j=0
    for bs in bb_arr:
        point_in_bbx = np.bitwise_and([uvs[:,0] >= bs.xmin], [(uvs[:,0]) < bs.xmax])
        point_in_bby = np.bitwise_and([uvs[:,1] >= bs.ymin], [(uvs[:,1]) < bs.ymax])
        point_in_bb = np.bitwise_and(point_in_bbx, point_in_bby)
        depth_bb = depth[np.squeeze(point_in_bb),:]
        if (j==0):
            depth_bb_coll = depth_bb
            j+=1
        else:
            depth_bb_coll = np.vstack((depth_bb_coll, depth_bb))
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "os_sensor"
        uvs_bb = uvs[np.squeeze(point_in_bb),:]
        closest_point = distance.cdist([(bs.xmax-bs.xmin) * 0.5 + bs.xmin,(bs.ymax-bs.ymin) * 0.5 + bs.ymin], uvs_bb).argmin()
        print(closest_point)

    pc = point_cloud2.create_cloud(header, fields, depth_bb_coll[:,:3])
    pub.publish(pc)



def add_ring_value():
    

    image_sub = message_filters.Subscriber('/rgb_publisher/color/image', Image)
    pcl_sub = message_filters.Subscriber('/lidar_0/m1600/pcl2', pc2)
    bboxes = message_filters.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes)
    ts = message_filters.ApproximateTimeSynchronizer([image_sub, pcl_sub, bboxes], 20, 0.2)
    ts.registerCallback(callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        add_ring_value()
    except rospy.ROSInterruptException:
        pass
