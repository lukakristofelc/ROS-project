#!/usr/bin/python3

import numpy as np
from scipy.spatial import distance
import rospy
import cv2
from homework6.msg import CylinderMsg
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped, Vector3, Pose
from std_msgs.msg import ColorRGBA
from nav_msgs.msg import OccupancyGrid
import PyKDL

class CylinderMarkers:
    def __init__(self):
        ros_map = rospy.wait_for_message("map",OccupancyGrid)
        self.frame = PyKDL.Frame(PyKDL.Rotation.Quaternion(ros_map.info.origin.orientation.x,ros_map.info.origin.orientation.y,ros_map.info.origin.orientation.z,ros_map.info.origin.orientation.w), PyKDL.Vector(ros_map.info.origin.position.x,ros_map.info.origin.position.y,ros_map.info.origin.position.z))
        self.inverse = self.frame.Inverse()
        self.map_size = (ros_map.info.height, ros_map.info.width)
        map_data = ros_map.data
        mapper = {-1:0, 0:0, 100:127}
        map_data = list(map(lambda a : mapper[a], map_data))
        map_data = np.reshape(map_data, self.map_size)
        map_data = map_data.astype("float64")
        self.map_data = map_data
        self.map_resolution = ros_map.info.resolution

        self.offset_dist = 5 # Change 

        self.cylinder_sub = rospy.Subscriber("/detected_cylinder_point", CylinderMsg, self.cylinder_callback)

        self.markers_pub = rospy.Publisher('cylinder_markers', MarkerArray, queue_size=1000)

        self.offset_pub = rospy.Publisher('cylinder_offsets', MarkerArray, queue_size=1000)

        self.num = 0
        self.centres = np.zeros((100,3))
        self.all_points = []
        self.color_centres = np.zeros((100,3))
        self.color_all_points = []
        self.offset_centres = np.zeros((100,3))
        self.offset_all_points = []

    def cylinder_callback(self, msg: CylinderMsg):
        point = np.array([msg.point.x,msg.point.y,msg.point.z])
        color = np.array([msg.r,msg.g,msg.b])
        offset = self.getOffset(point)
        d = distance.cdist([point],self.centres[:self.num,:])
        if self.num==0 or np.min(d)>0.5:
            self.all_points.append([])
            self.all_points[self.num].append(point)
            self.centres[self.num,:] = point
            self.color_all_points.append([])
            self.color_all_points[self.num].append(color)
            self.color_centres[self.num,:] = color
            self.offset_all_points.append([])
            self.offset_all_points[self.num].append(offset)
            self.offset_centres[self.num,:] = offset
            self.num += 1
        else:
            i = np.argmin(d)
            self.all_points[i].append(point)
            tp = np.array(self.all_points[i])
            self.centres[i,:] = np.mean(tp,axis = 0)
            self.color_all_points[i].append(color)
            tp = np.array(self.color_all_points[i])
            self.color_centres[i,:] = np.mean(tp,axis = 0)
            self.offset_all_points[i].append(offset)
            tp = np.array(self.offset_all_points[i])
            self.offset_centres[i,:] = np.mean(tp,axis = 0)

    def getOffset(self, point: np.array):
        point = np.array(list(self.inverse * PyKDL.Vector(point[0],point[1],0) / self.map_resolution)).astype(int)[:2]
        circle_filter = np.zeros_like(self.map_data)
        cv2.circle(circle_filter, tuple(point), 8, 255, -1)
        filterd_image = cv2.bitwise_and(self.map_data, self.map_data, mask = circle_filter.astype("int8"))
        indices = np.nonzero(filterd_image)
        y_offset = np.sum(indices[0] - point[1])
        x_offset = np.sum(indices[1] - point[0])
        angle = np.arctan2(y_offset, x_offset)
        offset = point - np.array([np.cos(angle), np.sin(angle)]) * self.offset_dist
        return np.array(list(self.frame * (PyKDL.Vector(offset[0], offset[1], 0) * self.map_resolution)))

    def cylinder_publisher(self):
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
            marker.text = "C{}".format(len(self.all_points[i]))
            marker.action = Marker.ADD
            marker.frame_locked = False
            marker.lifetime = rospy.Duration.from_sec(2)
            marker.id = i
            marker.scale = Vector3(0.5, 0.5, 0.5)
            marker.color = ColorRGBA(self.color_centres[i][0]/255, self.color_centres[i][1]/255, self.color_centres[i][2]/255, 1)
            marker_array.markers.append(marker)
        self.markers_pub.publish(marker_array)

    def offset_publisher(self):
        marker_array = MarkerArray()
        for i in range(self.num):
            pose = Pose()
            pose.position.x = self.offset_centres[i,0]
            pose.position.y = self.offset_centres[i,1]
            pose.position.z = self.offset_centres[i,2]
            pose.orientation.x = 1
            pose.orientation.y = 0
            pose.orientation.z = 0
            pose.orientation.w = 1

            marker = Marker()
            marker.header.stamp = rospy.Time(0)
            marker.header.frame_id = 'map'
            marker.pose = pose
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.frame_locked = False
            marker.lifetime = rospy.Duration.from_sec(2)
            marker.id = i
            marker.scale = Vector3(0.1, 0.1, 0.1)
            marker.color = ColorRGBA(0, 0, 1, 1)
            marker_array.markers.append(marker)
        self.offset_pub.publish(marker_array)


def main():
    rospy.loginfo("cylinder_markers started")
    rospy.init_node('cylinder_markers')
    cylinder_markers = CylinderMarkers()
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        cylinder_markers.cylinder_publisher()
        cylinder_markers.offset_publisher()
        r.sleep()

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
