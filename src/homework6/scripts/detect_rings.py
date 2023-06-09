#!/usr/bin/python3

import sys
import rospy
import cv2
import numpy as np
import tf2_geometry_msgs
import tf2_ros
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, Vector3, Pose
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from scipy.spatial import distance
import message_filters


class The_Ring:
    def __init__(self):

        # An object we use for converting images between ROS format and OpenCV format
        self.bridge = CvBridge()

        # A help variable for holding the dimensions of the image
        self.dims = (0, 0, 0)

        # Subscribe to the image and/or depth topic
        self.image_sub = message_filters.Subscriber("/camera/rgb/image_raw", Image)
        self.depth_sub = message_filters.Subscriber('/camera/depth/image_raw', Image)
        self.ts = message_filters.TimeSynchronizer([self.image_sub,self.depth_sub],1000)
        self.ts.registerCallback(self.image_callback)
        # self.depth_sub = rospy.Subscriber("/camera/depth_registered/image_raw", Image, self.depth_callback)

        # Publiser for the visualization markers
        self.markers_pub = rospy.Publisher('ring_markers', MarkerArray, queue_size=1000)

        # Object we use for transforming between coordinate frames
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)

        self.num = 0
        self.centres = np.zeros((100,3))
        self.all_points = []
        self.color_centres = np.zeros((100,3))
        self.color_all_points = []


    def get_pose(self,e,dist,stamp):
        # Calculate the position of the detected ellipse

        k_f = 554 # kinect focal length in pixels

        elipse_x = self.dims[1] / 2 - e[0][0]
        elipse_y = self.dims[0] / 2 - e[0][1]

        angle_to_target = np.arctan2(elipse_x,k_f)

        # Get the angles in the base_link relative coordinate system
        x,y = dist*np.cos(angle_to_target), dist*np.sin(angle_to_target)

        # Define a stamped message for transformation - in the "camera rgb frame"
        point_s = PointStamped()
        point_s.point.x = -y
        point_s.point.y = 0
        point_s.point.z = x
        point_s.header.frame_id = "camera_rgb_optical_frame"
        point_s.header.stamp = stamp

        # Get the point in the "map" coordinate system
        try:
            point_world = self.tf_buf.transform(point_s, "map")
            pose = np.array([point_world.point.x,point_world.point.y,point_world.point.z])
            if np.isnan(pose).any():
                return None
            else:
                return pose
        except Exception as e:
            print(str(e))
            return None


    def image_callback(self, data, depth_img):
        #print('I got a new image!')

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        depth_time = depth_img.header.stamp
        depth_image = self.bridge.imgmsg_to_cv2(depth_img, "32FC1")

        # Set the dimensions of the image
        self.dims = cv_image.shape

        # Tranform image to gayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Do histogram equlization
        img = cv2.equalizeHist(gray)

        # Binarize the image
        #ret, thresh = cv2.threshold(img, 50, 255, 0)
        ret, thresh = cv2.threshold(img, 50, 255, cv2.THRESH_OTSU)

        # Extract contours
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        # Example how to draw the contours
        # cv2.drawContours(img, contours, -1, (255, 0, 0), 3)

        # Fit elipses to all extracted contours
        elps = []
        for cnt in contours:
            if cnt.shape[0] >= 20:
                ellipse = cv2.fitEllipse(cnt)
                elps.append(ellipse)


        # Find two elipses with same centers
        candidates = []
        for n in range(len(elps)):
            for m in range(n + 1, len(elps)):
                e1 = elps[n]
                e2 = elps[m]
                dist = np.sqrt(((e1[0][0] - e2[0][0]) ** 2 + (e1[0][1] - e2[0][1]) ** 2))
                #             print dist
                if dist < 5:
                    candidates.append((e1,e2))

        #print("Processing is done! found", len(candidates), "candidates for rings")

        # Extract the depth from the depth image
        fil_ring = np.zeros((cv_image.shape[0],cv_image.shape[1]))
        fil_inner = np.zeros((cv_image.shape[0],cv_image.shape[1]))
        filtered_image = np.copy(cv_image)
        for c in candidates:

            # the centers of the ellipses
            e1 = c[1] if c[0][1][0]*c[0][1][1] < c[1][1][0]*c[1][1][1] else c[0]
            e2 = c[0] if c[0][1][0]*c[0][1][1] < c[1][1][0]*c[1][1][1] else c[1]

            # extract ellipse area
            cv2.ellipse(fil_ring, e1, (255, 255, 255), -1)
            cv2.ellipse(fil_ring, e2, (0, 0, 0), -1)

            cv2.ellipse(fil_inner, e2, (255, 255, 255), -1)

            ring = depth_image[fil_ring.astype(bool)]
            inner = depth_image[fil_inner.astype(bool)]

            depth_ring = float(np.nanmean(ring))
            depth_inner = float(np.nanmean(inner))

            nan_ring = np.count_nonzero(np.isnan(ring))
            nan_inner = np.count_nonzero(np.isnan(inner))

            # get ring color
            ring_color_area = cv_image[fil_ring.astype(bool)]
            ring_color = np.nanmean(ring_color_area,axis=0)

            # prepere detection info to show in a window
            cv2.ellipse(cv_image, e1, (0, 0, 255), 2)
            cv2.ellipse(cv_image, e2, (0, 0, 255), 2)
            text = "{:.4f} {:.0f} {:.0f}".format(depth_ring - depth_inner, nan_ring/ring.size * 100, nan_inner/inner.size * 100)
            text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
            pos = (int(e1[0][0] - text_size[0][0]/2), int(e1[0][1] + (e1[1][0]+e1[1][1])/4* 1.8))
            cv2.putText(cv_image, text, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            if nan_ring/ring.size > 0.4 or nan_inner/inner.size < 0.8:
                continue

            point = self.get_pose(e1, depth_ring, depth_time)

            if point is not None:
                d = distance.cdist([point],self.centres[:self.num,:])
                if self.num==0 or np.min(d)>0.4:
                    self.all_points.append([])
                    self.all_points[self.num].append(point)
                    self.centres[self.num,:] = point
                    self.color_all_points.append([])
                    self.color_all_points[self.num].append(ring_color)
                    self.color_centres[self.num,:] = ring_color
                    self.num += 1
                else:
                    i = np.argmin(d)
                    self.all_points[i].append(point)
                    tp = np.array(self.all_points[i])
                    self.centres[i,:] = np.mean(tp,axis = 0)
                    self.color_all_points[i].append(ring_color)
                    tp = np.array(self.color_all_points[i])
                    self.color_centres[i,:] = np.mean(tp,axis = 0)

        marker_array = MarkerArray()
        for i in range(self.num):
            pose = Pose()
            pose.position.x = self.centres[i,0]
            pose.position.y = self.centres[i,1]
            pose.position.z = self.centres[i,2]
            pose.orientation.x = 1
            pose.orientation.y = 0
            pose.orientation.z = 0
            pose.orientation.w = 1

            marker = Marker()
            marker.header.stamp = rospy.Time(0)
            marker.header.frame_id = 'map'
            marker.pose = pose
            marker.type = Marker.TEXT_VIEW_FACING
            marker.text = "R{}".format(len(self.all_points[i]))
            marker.action = Marker.ADD
            marker.frame_locked = False
            marker.lifetime = rospy.Duration.from_sec(10)
            marker.id = i
            marker.scale = Vector3(0.5, 0.5, 0.5)
            marker.color = ColorRGBA(self.color_centres[i][2]/255, self.color_centres[i][1]/255, self.color_centres[i][0]/255, 1)
            marker_array.markers.append(marker)

        self.markers_pub.publish(marker_array)

        cv2.imshow("Image window",cv_image)
        cv2.waitKey(1)

    def depth_callback(self,data):

        try:
            depth_image = self.bridge.imgmsg_to_cv2(data, "16UC1")
        except CvBridgeError as e:
            print(e)

        # Do the necessairy conversion so we can visuzalize it in OpenCV
        image_1 = depth_image / 65536.0 * 255
        image_1 =image_1/np.max(image_1)*255

        image_viz = np.array(image_1, dtype= np.uint8)

        #cv2.imshow("Depth window", image_viz)
        #cv2.waitKey(1)


def main():

    rospy.init_node("detect_rings")
    ring_finder = The_Ring()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
