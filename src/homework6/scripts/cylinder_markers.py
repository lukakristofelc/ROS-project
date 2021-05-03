#!/usr/bin/python3

import numpy as np
from scipy.spatial import distance
import rospy
from homework6.msg import CylinderMsg
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped, Vector3, Pose
from std_msgs.msg import ColorRGBA

class CylinderMarkers:
    def __init__(self):
        self.cylinder_sub = rospy.Subscriber("/detected_cylinder_point", CylinderMsg, self.cylinder_callback)

        self.markers_pub = rospy.Publisher('cylinder_markers', MarkerArray, queue_size=1000)

        self.num = 0
        self.centres = np.zeros((100,3))
        self.all_points = []
        self.color_centres = np.zeros((100,3))
        self.color_all_points = []

    def cylinder_callback(self, msg: CylinderMsg):
        point = np.array([msg.point.x,msg.point.y,msg.point.z])
        color = np.array([msg.r,msg.g,msg.b])
        d = distance.cdist([point],self.centres[:self.num,:])
        if self.num==0 or np.min(d)>0.5:
            self.all_points.append([])
            self.all_points[self.num].append(point)
            self.centres[self.num,:] = point
            self.color_all_points.append([])
            self.color_all_points[self.num].append(color)
            self.color_centres[self.num,:] = color
            self.num += 1
        else:
            i = np.argmin(d)
            self.all_points[i].append(point)
            tp = np.array(self.all_points[i])
            self.centres[i,:] = np.mean(tp,axis = 0)
            self.color_all_points[i].append(color)
            tp = np.array(self.color_all_points[i])
            self.color_centres[i,:] = np.mean(tp,axis = 0)

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


def main():
    rospy.loginfo("cylinder_markers started")
    rospy.init_node('cylinder_markers')
    cylinder_markers = CylinderMarkers()
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        cylinder_markers.cylinder_publisher()
        r.sleep()

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
