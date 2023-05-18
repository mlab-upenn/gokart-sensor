import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from collections import namedtuple
import numpy as np
import threading
import cv2
import time
import math


BEV_WIDTH = 500
BEV_HEIGHT = 600
CAR_ORIGIN = [int(BEV_WIDTH/2), BEV_HEIGHT] # in bev

IMG_SHAPE = (1080, 1920)

PX_DIST = 0.04
IMG_COOR = [[248, 133], [794, 536], [1065, 533], [1604, 100]]
#WORLD_COOR = [[0.614, 2.56], [0.614, 6.6], [-0.614, 6.6], [-0.614, 2.56]]
BEV_COOR =[[int(CAR_ORIGIN[0] + 0.614 / PX_DIST), int(CAR_ORIGIN[1] - 2.56 / PX_DIST)], 
           [int(CAR_ORIGIN[0] + 0.55 / PX_DIST), int(CAR_ORIGIN[1] - 6.6 / PX_DIST)],
           [int(CAR_ORIGIN[0] - 0.55 / PX_DIST), int(CAR_ORIGIN[1] - 6.6 / PX_DIST)],
           [int(CAR_ORIGIN[0] - 0.614 / PX_DIST), int(CAR_ORIGIN[1] - 2.56 / PX_DIST)]]

MAT = cv2.getPerspectiveTransform(np.float32(IMG_COOR) * 1.059, np.float32(BEV_COOR))

WHITE_BOX_DETECTION = False
BOX_MINIMUM_SIZE = 6000

ANGLE_INCREMENT = 0.5 #deg
ANGULAR_RANGE = 180 #deg
MAX_RANGE = 30.0 #meter

class cam_to_lidar(Node):
    def __init__(self):
        # here, super().__init__(<node_name>), while the node_name should be the same as provided in launch yaml file
        super().__init__("cam_to_lidar_node")
        
        # subscribe and publish
        self.dt = 0.1
        self.data= np.zeros(1)
        # self.get_logger().info("node initialized, dt = %f" % self.dt)
        self.image_sub = self.create_subscription(Image, "/color/image", self.image_cb, qos_profile=qos_profile_sensor_data)
        self.laser_pub = self.create_publisher(LaserScan, "/cam_laser", 10)
        # self.timer = self.create_timer(self.dt, self.main_update)
        self.msgLock = threading.Lock()


    def image_cb(self, msg: Image):
        # print("dljcnv")
        img_rbg_array = np.array(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        ts = time.time() 
        self.grass_edge_detection(img_rbg_array)
        # print(self.data)
        pub_laser = LaserScan()
        pub_laser.header.frame_id = "os_sensor"
        pub_laser.header.stamp = msg.header.stamp
        pub_laser.angle_increment = 0.5*math.pi/180.0
        pub_laser.angle_max = 1.57
        pub_laser.angle_min = -1.57
        pub_laser.range_max = 50.0
        pub_laser.range_min = 1.0
        pub_laser.scan_time = 0.1
        pub_laser.ranges = self.data
        self.laser_pub.publish(pub_laser)

        # print('computation time:', '{:.3f} sec'.format(time.time() - ts)) 
        if cv2.waitKey(1) & 0xFF == ord('q'):
            return

    # def ekf_update(self):
    #     print("")
    def grass_edge_detection(self, img):
        img = cv2.blur(img, (10, 10))
        b,g,r = cv2.split(img)
        grass_gray = 1 - (b - 0.6 * g)
        ret, grass = cv2.threshold(grass_gray, 1, 255, cv2.THRESH_BINARY)
        kernel = np.ones((5, 5),np.uint8)
        grass = cv2.morphologyEx(grass, cv2.MORPH_OPEN, kernel)
        grass = cv2.morphologyEx(grass, cv2.MORPH_CLOSE, kernel)
        
        bev_grass = cv2.warpPerspective(grass, MAT, (BEV_WIDTH, BEV_HEIGHT))
        bev_img = cv2.warpPerspective(img, MAT, (BEV_WIDTH, BEV_HEIGHT))
        # copy the grayscale BEV map and fills its projection regions by setting all of the non-zero pixels to white (255)
        fov_edge = cv2.cvtColor(bev_img, cv2.COLOR_BGR2GRAY)
        fov_edge[fov_edge > 0] = 255
        # perform Canny edge detection to get the projection boarder edges
        fov_edge = cv2.Canny(fov_edge, 60, 130) # 60, 130
        # thickening(7 extra pixels) the previously detemined boarder edges
        fov_edge = cv2.dilate(fov_edge, np.ones((7,7), np.uint8))
        white_boxes = self.box_detection(bev_img)
        self.data = self.get_dist_array(bev_grass, white_boxes, fov_edge)


        self.draw_dist_array(bev_img, self.data)
        #print(data)
        
        # print(img.shape)
        # view_img = img.reshape((540, 960))
        deb = False
        if deb:
            cv2.imshow("src", img)
            cv2.imshow("grass_gray", grass_gray)
            cv2.imshow("bev_img", bev_img)

    def box_detection(self, bev):
        gray = cv2.cvtColor(bev, cv2.COLOR_BGR2GRAY)
        ret, white_objects = cv2.threshold(gray, 180, 255, cv2.THRESH_BINARY)
        contours, hierarchy = cv2.findContours(white_objects, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        white_boxes = np.zeros((white_objects.shape[0], white_objects.shape[1]), dtype = "uint8")
        filtered_contours = []
        
        for contour in contours:
            M = cv2.moments(contour)
            if M["m00"] != 0:

                area = cv2.contourArea(contour)
                (x, y, w, h) = cv2.boundingRect(contour)
                ratio = max(w,h)/min(w,h) #distance of the primary axis
                
                #size filter
                if area > BOX_MINIMUM_SIZE:
                    filtered_contours.append(contour)

            cv2.drawContours(white_boxes, filtered_contours,-1, 255, -1)
        #cv2.imshow('white_boxes', white_boxes)  
        return white_boxes
        
    def get_dist_array(self, bev, white_boxes, fov_edge):
        num_data = int(ANGULAR_RANGE / ANGLE_INCREMENT)
        data = [MAX_RANGE] * num_data
        if WHITE_BOX_DETECTION:
            bev += white_boxes

        for i in range(num_data):
            ray_dist = 1
            ray_angle = i * ANGLE_INCREMENT
            while ray_dist * PX_DIST <= MAX_RANGE and ray_angle > 40 and ray_angle < 140:
                x = int(CAR_ORIGIN[0] + ray_dist * math.cos(ray_angle * math.pi / 180))
                y = int(CAR_ORIGIN[1] - ray_dist * math.sin(ray_angle * math.pi / 180))
                if x >= BEV_WIDTH or x < 0 or y >= BEV_HEIGHT or y < 0:
                    break
                if bev[y][x] > 100:
                    if fov_edge[y][x] == 0:
                        data[i] = ray_dist * PX_DIST
                    break
                ray_dist += 2
        
        return data

    def draw_dist_array(self, bev, dist_array):
        ray_angle = 0
        for dist in dist_array:
            x = int(CAR_ORIGIN[0] + dist * math.cos(ray_angle * math.pi / 180) / PX_DIST)
            y = int(CAR_ORIGIN[1] - dist * math.sin(ray_angle * math.pi / 180) / PX_DIST)
            cv2.circle(bev, (x, y), 3, (0, 255, 0), -1)
            ray_angle += ANGLE_INCREMENT
    
    def custom_fit_ellipse(self, edges):
        y, x = np.nonzero(edges)
        if len(x) == 0:
            return 0
        x = x - np.mean(x)
        y = y - np.mean(y)
        coords = np.vstack([x, y])
        cov = np.cov(coords)
        evals, evecs = np.linalg.eig(cov)
        sort_indices = np.argsort(evals)[::-1]
        x_v1, y_v1 = evecs[:, sort_indices[0]]  # Eigenvector with largest eigenvalue
        #x_v2, y_v2 = evecs[:, sort_indices[1]]
        angle = np.arctan((x_v1)/(y_v1)) * 180 / math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = cam_to_lidar()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()