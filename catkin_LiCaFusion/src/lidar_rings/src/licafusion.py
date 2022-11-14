import rospy
from sensor_msgs.msg import PointCloud2 as pc2
from sensor_msgs.msg import PointField
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import message_filters
import numpy as np
import ros_numpy
import darknet_ros_msgs
from darknet_ros_msgs.msg import BoundingBoxes
import cv2
from cv_bridge import CvBridge
from sensor_msgs import point_cloud2

rospy.init_node('licafusion', anonymous=True)
pub = rospy.Publisher("/lidar_0/m1600/pcl2_XYZIR", pc2, queue_size=10)
pub2 = rospy.Publisher("/res_img", Image, queue_size=10)
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
    # rospy.loginfo("1")
    pcl_XYZIR = pcl
    xyz_arr = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pcl)
    # print(xyz_arr.shape)
    point_arr = np.ones((xyz_arr.shape[0]))
    h_array = np.column_stack((xyz_arr, point_arr))
    # rospy.loginfo("%s", pcl_XYZIR.fields)
    c_t_l = np.array([[0.0190983 , -0.999815 ,0.00232702 , 0.0440449],
                    [-0.193816 ,-0.0059855 ,  -0.98102 ,  0.142687],
                    [ 0.980852 , 0.0182848 , -0.193894 ,-0.0378563],
                    [0,0,0,1]])
    proj_mat = np.array([[809.148438 , 0.0 ,328.785593 , 0.0],
                    [0.0 ,808.892273 ,  239.680971 ,  0.0],
                    [ 0.0 , 0.0 , 1.0 ,0.0]])
    # l_t_c = np.linalg.inv(c_t_l)
    cam_points = c_t_l@h_array.T
    # print(cam_points[:,0])
    uvs = project_points(proj_mat, cam_points.T)
    # print(uvs[:4, :])

    point_in_fovx = np.bitwise_and([uvs[:,0] >= 0], [(uvs[:,0]) < image.width])
    point_in_fovy = np.bitwise_and([uvs[:,1] >= 0], [(uvs[:,1]) < image.height])
    point_in_fov = np.bitwise_and(point_in_fovx, point_in_fovy)
    uvs = uvs[np.squeeze(point_in_fov),:]
    depth = h_array[np.squeeze(point_in_fov),:]

    # print(image.width)
    # print(image.height)
    # print(uvs.shape)
    color_red = (0, 0, 255)
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
          PointField('y', 4, PointField.FLOAT32, 1),
          PointField('z', 8, PointField.FLOAT32, 1)
          ]

    cv_img = bridge.imgmsg_to_cv2(image, "passthrough")
    bb_arr = bb.bounding_boxes
    # depth_bb_coll = np.array([])
    j=0
    for bs in bb_arr:
        point_in_bbx = np.bitwise_and([uvs[:,0] >= bs.xmin], [(uvs[:,0]) < bs.xmax])
        point_in_bby = np.bitwise_and([uvs[:,1] >= bs.ymin], [(uvs[:,1]) < bs.ymax])
        point_in_bb = np.bitwise_and(point_in_bbx, point_in_bby)
        uvs_bb = uvs[np.squeeze(point_in_bb),:]
        depth_bb = depth[np.squeeze(point_in_bb),:]
        # print(depth_bb.shape)
        if (j==0):
            depth_bb_coll = depth_bb
            j+=1
        else:
            depth_bb_coll = np.vstack((depth_bb_coll, depth_bb))
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "lidar_0"
        for uv in uvs_bb:
            cv2.circle(cv_img, tuple(uv), 3, color_red, -1)

    pc = point_cloud2.create_cloud(header, fields, depth_bb_coll[:,:3])
    pub.publish(pc)

    cv_img = bridge.cv2_to_imgmsg(cv_img, "passthrough")
    pub2.publish(cv_img)



def add_ring_value():
    
    
    # rospy.Subscriber('/lidar_0/m1600/pcl2', PointCloud2, test_cb)
    image_sub = message_filters.Subscriber('/usb_cam/image_raw', Image)
    pcl_sub = message_filters.Subscriber('/lidar_0/m1600/pcl2', pc2)
    bboxes = message_filters.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes)
    ts = message_filters.ApproximateTimeSynchronizer([image_sub, pcl_sub, bboxes], 5, 0.1)
    ts.registerCallback(callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        add_ring_value()
    except rospy.ROSInterruptException:
        pass
