import rospy
from sensor_msgs.msg import PointCloud2 as pc2
from sensor_msgs.msg import Image
import message_filters
import numpy as np
from darknet_ros_msgs.msg import BoundingBoxes
from darknet_ros_msgs.msg import BoundingBox
from cv_bridge import CvBridge
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from cv_bridge import CvBridge
import cv2



rospy.init_node('latefusion_validation', anonymous=True)


pub_img = rospy.Publisher("/res_img", Image, queue_size=10)


bridge = CvBridge()

def IoU(marker, darknet):
    # calculate the intersection area
    x_min = max(marker.xmin, darknet.xmin)
    y_min = max(marker.ymin, darknet.ymin)
    x_max = min(marker.xmax, darknet.xmax)
    y_max = min(marker.ymax, darknet.ymax)
    if x_max < x_min or y_max < y_min:
        return 0.0
    intersection_area = (x_max - x_min) * (y_max - y_min)

    # calculate the union area
    marker_area = (marker.xmax - marker.xmin) * (marker.ymax - marker.ymin)
    darknet_area = (darknet.xmax - darknet.xmin) * (darknet.ymax - darknet.ymin)
    union_area = marker_area + darknet_area - intersection_area

    # calculate the IoU
    iou = intersection_area / union_area
    assert iou >= 0.0
    assert iou <= 1.0
    return iou



def check_overlap(marker, darknet):
    if marker.xmin > darknet.xmax or marker.xmax < darknet.xmin or marker.ymin > darknet.ymax or marker.ymax < darknet.ymin:
        return False
    else:
        return True



def project_points(projection_matrix, points):
    assert points.shape[-1] == 4
    points = points[points[:,2] > 0.0]
    uvs = np.matmul(projection_matrix,points.T)
    uvs = uvs/uvs[2,:]
    
    uvs = uvs[0:2,:]
    uvs = uvs.astype(np.int)

    return uvs.T

def callback(marker_array_openpc, bb, image):


    # TODO: define lidar to camera projection matrix and camera matrix (default values for Velodyne M1600 and OAK-D fusion and OAK-D camera parameters)
    l_t_c = np.array([[-0.00423105, -0.99987, -0.0155724, 0.09296],
                      [0.0117262, 0.0155218, -0.999811, -0.0837121],
                      [0.999922, -0.00441285, 0.011659, -0.0289284],
                      [0, 0, 0, 1]])
    proj_mat = np.array([[1442.193090, 0.0, 943.936435, 0.0],
                         [0.0, 1440.905704, 542.578706, 0.0],
                         [0.0, 0.0, 1.0, 0.0]])

    # initiate marker array
    marker_array = MarkerArray()

    cv_img = bridge.imgmsg_to_cv2(image, "rgb8")


    for marker_single in marker_array_openpc.markers:
        marker_array_lidar = np.empty((28,4))
        i = 0
        for marker_bb_single in marker_single.points:
            marker_array_lidar[i,0] = marker_bb_single.x
            marker_array_lidar[i,1] = marker_bb_single.y
            marker_array_lidar[i,2] = marker_bb_single.z
            marker_array_lidar[i,3] = 1.0
            i += 1

        # convert marker array to camera 3D frame
        marker_array_cam_3D = l_t_c @ marker_array_lidar.T

        # project 3D bounding boxes to the image plane
        uvs = project_points(proj_mat, marker_array_cam_3D.T)
        print(uvs)
        print(uvs.max(axis=0))
        print(uvs.min(axis=0))
        marker_2D_bb = BoundingBox()
        marker_2D_bb.xmax, marker_2D_bb.ymax = uvs.max(axis=0)
        marker_2D_bb.xmin, marker_2D_bb.ymin = uvs.min(axis=0)

        # check if the projected 3D bounding box is within the image FOV
        if marker_2D_bb.xmin < 0 or marker_2D_bb.ymin < 0 or marker_2D_bb.xmax > 1920 or marker_2D_bb.ymax > 1080:
            continue

        for bs in bb.bounding_boxes:
            if check_overlap(bs, marker_2D_bb):
                #print the x and y min and max of the bounding box
                print(bs.xmin, bs.ymin, bs.xmax, bs.ymax)
                #print the x and y min and max of the projected 3D bounding box
                print(marker_2D_bb.xmin, marker_2D_bb.ymin, marker_2D_bb.xmax, marker_2D_bb.ymax)
                print(type(cv_img))
                # OpenCV rectangle of the bounding box
                try:
                    cv2.rectangle(cv_img, (int(bs.xmin), int(bs.ymin)), (int(bs.xmax), int(bs.ymax)), (0, 255, 0), 2)
                except:
                    print("error")
                    continue
                # OpenCV rectangle of the projected 3D bounding box
                try:
                    cv2.rectangle(cv_img, (marker_2D_bb.xmin, marker_2D_bb.ymin), (marker_2D_bb.xmax, marker_2D_bb.ymax), (0, 0, 255), 2)
                except:
                    print("error")
                    continue

                print("overlap")
                iou = IoU(bs, marker_2D_bb)

                print(iou)
                if iou > 0.3:
                    print("detection")

                cv_img = bridge.cv2_to_imgmsg(cv_img, "rgb8")
                pub_img.publish(cv_img)



def late_fusion_validation():
    
    
    # approximate time synchroniser for MarkerArray and BoundingBoxes
    # TODO: change marker topic to /detect_3dbox for OpenPCDet and /cluster_markers for Geometric LiDAR Detection
    marker_sub = message_filters.Subscriber('/detect_3dbox', MarkerArray)
    bb_sub = message_filters.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes)
    image_sub = message_filters.Subscriber('/rgb_publisher/color/image', Image)
    ts = message_filters.ApproximateTimeSynchronizer([marker_sub, bb_sub, image_sub], 5, 0.1, allow_headerless=True)
    ts.registerCallback(callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        late_fusion_validation()
    except rospy.ROSInterruptException:
        pass
