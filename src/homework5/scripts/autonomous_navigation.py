#!/usr/bin/env python

import rospy
import numpy as np
import cv2
import tf2_geometry_msgs
import actionlib
import time
from nav_msgs.msg import OccupancyGrid, Odometry
from nav_msgs.srv import GetPlan, GetPlanResponse
from scipy.spatial import distance
from geometry_msgs.msg import TransformStamped, PoseStamped, Pose, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from python_tsp.exact import solve_tsp_dynamic_programming

map_valus={-1:0,0:255,100:0}
elements = [[[1,0,1],[0,1,0],[0,1,0]],[[0,1,0],[0,1,1],[1,0,0]],[[0,0,1],[1,1,0],[0,0,1]],[[1,0,0],[0,1,1],[0,1,0]],[[0,1,0],[0,1,0],[1,0,1]],[[0,0,1],[1,1,0],[0,1,0]],[[1,0,0],[0,1,1],[1,0,0]],[[0,1,0],[1,1,0],[0,0,1]],[[1,0,0],[0,1,0],[1,0,1]],[[1,0,1],[0,1,0],[1,0,0]],[[1,0,1],[0,1,0],[0,0,1]],[[0,0,1],[0,1,0],[1,0,1]]]
elements = np.array(elements)

def getPoints(map_map: OccupancyGrid):
    ros_map_data = map_map.data  
    ros_map_data = map(lambda a : map_valus[a],ros_map_data)
    ros_map_data = np.array(list(ros_map_data))
    size = (map_map.info.height,map_map.info.width)
    ros_map_data = np.resize(ros_map_data,size).astype("uint8")

    img = ros_map_data
    skel = np.zeros(img.shape, np.uint8)

    element = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
    element2 = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))

    while True:
        open = cv2.morphologyEx(img, cv2.MORPH_OPEN, element)
        temp = cv2.subtract(img, open)
        eroded = cv2.erode(img, element2)
        skel = cv2.bitwise_or(skel,temp)
        img = eroded.copy()
        if cv2.countNonZero(img)==0:
            break

    #cv2.imshow("Skeleton",skel)
    #cv2.waitKey(0)

    intersections = np.zeros(img.shape, np.uint8)
    for element in elements:
        temp = cv2.morphologyEx(skel,cv2.MORPH_HITMISS,element)
        intersections = cv2.bitwise_or(intersections,temp)

    intersection_coords = np.transpose(np.nonzero(intersections))
    centres = []
    all_points = []

    for intersection_coord in intersection_coords:
        if len(centres)!=0:
            d = distance.cdist([intersection_coord],centres)
        if len(centres)==0 or np.min(d)>10:
            all_points.append([])
            all_points[-1].append(intersection_coord)
            centres.append(intersection_coord)
        else:
            i = np.argmin(d)
            all_points[i].append(intersection_coord)
            centres[i] = np.mean(all_points[i],axis=0)

    centres = np.array(centres).astype("int")
    centres = np.flip(centres,axis=1)
    return centres

def transformCoordinates(ros_map: OccupancyGrid, centres: np.array):
    map_transform = TransformStamped()
    map_transform.transform.translation.x = ros_map.info.origin.position.x
    map_transform.transform.translation.y = ros_map.info.origin.position.y
    map_transform.transform.translation.z = ros_map.info.origin.position.z
    map_transform.transform.rotation.x = ros_map.info.origin.orientation.x
    map_transform.transform.rotation.y = ros_map.info.origin.orientation.y
    map_transform.transform.rotation.z = ros_map.info.origin.orientation.z
    map_transform.transform.rotation.w = ros_map.info.origin.orientation.w

    transformedPoints=[]
    for centre in centres:
        point = tf2_geometry_msgs.PointStamped()
        point.point.x = centre[0] * ros_map.info.resolution
        point.point.y = centre[1] * ros_map.info.resolution
        point.point.z = 0
        transformedPoint = tf2_geometry_msgs.do_transform_point(point,map_transform)
        transformedPoints.append([transformedPoint.point.x,transformedPoint.point.y])
    return np.array(transformedPoints)

def findDistance(start_point: np.array, end_point: np.array):
    rospy.wait_for_service("move_base/make_plan")
    get_plan = rospy.ServiceProxy('move_base/make_plan', GetPlan)
    
    start = PoseStamped()
    start.header.seq = 0
    start.header.frame_id = "map"
    start.header.stamp = rospy.Time(0)
    start.pose.position.x = start_point[0]
    start.pose.position.y = start_point[1]

    goal = PoseStamped()
    goal.header.seq = 0
    goal.header.frame_id = "map"
    goal.header.stamp = rospy.Time(0)
    goal.pose.position.x = end_point[0]
    goal.pose.position.y = end_point[1]

    path: GetPlanResponse = get_plan(start, goal, .5)
    path = np.array(list(map(lambda point : [point.pose.position.x,point.pose.position.y], path.plan.poses)))
    distance = np.sum(np.square(path[1:,0]-path[:-1,0])+np.square(path[1:,1]-path[:-1,1]))

    return distance

def findShortestPath(transformedPoints: np.array):
    num_of_points = transformedPoints.shape[0]
    distance_matrix = np.zeros((num_of_points,num_of_points))
    for i, point1 in enumerate(transformedPoints):
        for j, point2 in enumerate(transformedPoints):
            if i < j:
                distance_matrix[i,j] = findDistance(point1, point2)
    distance_matrix = np.maximum( distance_matrix, distance_matrix.transpose() )
    permutation, distance = solve_tsp_dynamic_programming(distance_matrix)
    return permutation


def navigate(transformedPoints: np.array):
    odom: Odometry = rospy.wait_for_message("/odom", Odometry)
    action_client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    action_client.wait_for_server()

    position = [odom.pose.pose.position.x, odom.pose.pose.position.y]
    transformedPoints = np.insert(transformedPoints,0,[position],axis=0)
    transformedPoints = transformedPoints[findShortestPath(transformedPoints)]
    transformedPoints = transformedPoints[1:,:]
    print(transformedPoints)
    
    #rate = rospy.Rate(1)
    #while not rospy.is_shutdown():
    #    drawMarkers(transformedPoints)
    #    rate.sleep()
    
    ring_num = 0
    cylinder_num = 0

    while len(transformedPoints) > 0:
        goal_point = transformedPoints.pop(0)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = goal_point[0]
        goal.target_pose.pose.position.y = goal_point[1]
        goal.target_pose.pose.orientation.z = 0
        goal.target_pose.pose.orientation.w = 1
        goal.target_pose.header.stamp = rospy.Time.now()

        action_client.send_goal(goal)
        while action_client.get_state() in [0,1]:
            time.sleep(1)

        goal.target_pose.pose.orientation.z = 1
        goal.target_pose.pose.orientation.w = 0
        goal.target_pose.header.stamp = rospy.Time.now()

        action_client.send_goal(goal)

        while action_client.get_state() in [0,1]:
            ring_markers:MarkerArray = rospy.wait_for_message("ring_markers", MarkerArray)
            cylinder_markers:MarkerArray = rospy.wait_for_message("cylinder_markers", MarkerArray)

            if len(ring_markers)>ring_num:
                ring_goals = = ring_markers[ring_num:]
                for ring_goal in ring_goals:
                    transformedPoints.insert(0, [ring_goal.pose.position.x, ring_goal.pose.position.y])
                ring_num=len(ring_goals)

            if len(cylinder_markers)>cylinder_num:
                cylinder_goals = = cylinder_markers[cylinder_num:]
                for cylinder_goal in cylinder_goals:
                    transformedPoints.insert(0, [cylinder_goal.pose.position.x, cylinder_goal.pose.position.y])
                cylinder_num=len(cylinder_goals)
            time.sleep(1)
        

def showCentresImage(ros_map_data: np.array, centres: np.array):
    for i,centre in enumerate(centres):
        #cv2.circle(ros_map_data,tuple(centre),3,(0,0,255))
        cv2.putText(ros_map_data,str(i+1),tuple(centre),cv2.FONT_HERSHEY_SIMPLEX,0.4,(0,0,255))

    cv2.imshow("Intersections",ros_map)
    cv2.waitKey(0)

def drawMarkers(transformedPoints: np.array):
    pub = rospy.Publisher('prospective_goals', MarkerArray, queue_size=1000)
    marker_array = MarkerArray()
    for i,transformedPoint in enumerate(transformedPoints):
        marker = Marker()
        marker.header.stamp = rospy.Time(0)
        marker.header.frame_id = 'map'

        pose = Pose()
        pose.position.x = transformedPoint[0]
        pose.position.y = transformedPoint[1]
        pose.position.z = 0.0

        marker.pose = pose
        marker.type = Marker.TEXT_VIEW_FACING
        marker.text = "{}".format(i+1)
        marker.action = Marker.ADD
        marker.frame_locked = False
        marker.lifetime = rospy.Duration.from_sec(10)
        marker.id = i
        marker.scale = Vector3(0.5, 0.5, 0.5)
        marker.color = ColorRGBA(1, 0, 0, 1)
        marker_array.markers.append(marker)
    pub.publish(marker_array)

if __name__ == "__main__":
    rospy.init_node("autonomous_navigation")
    ros_map = rospy.wait_for_message("map",OccupancyGrid)
    centres: np.array = getPoints(ros_map)
    transformedPoints = transformCoordinates(ros_map, centres)
    navigate(transformedPoints)

    #rate = rospy.Rate(1)
    #while not rospy.is_shutdown():
    #    drawMarkers(transformedPoints[np.argsort(distances)])
    #    rate.sleep()

