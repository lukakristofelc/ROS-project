#!/usr/bin/python3

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
from std_msgs.msg import String
from homework4.msg import FaceGoals, FaceGoalsArray
from sound_play.libsoundplay import SoundClient

map_valus={-1:0,0:255,100:0}
elements = [[[1,0,1],[0,1,0],[0,1,0]],[[0,1,0],[0,1,1],[1,0,0]],[[0,0,1],[1,1,0],[0,0,1]],[[1,0,0],[0,1,1],[0,1,0]],[[0,1,0],[0,1,0],[1,0,1]],[[0,0,1],[1,1,0],[0,1,0]],[[1,0,0],[0,1,1],[1,0,0]],[[0,1,0],[1,1,0],[0,0,1]],[[1,0,0],[0,1,0],[1,0,1]],[[1,0,1],[0,1,0],[1,0,0]],[[1,0,1],[0,1,0],[0,0,1]],[[0,0,1],[0,1,0],[1,0,1]]]
elements = np.array(elements)
ring_num = 0
cylinder_num = 0
attackedHumans = []
face_goals_num = 0

colors = {'red': (35.00,24.00,11.00),
          'green': (54.92,-38.11,31.68),
          'blue': (21.00,0.00,-30.00),
          'yellow': (38.96,-7.16,27.26),
          'orange': (67.62,45.96,74.74),
          'white': (100.00,0.00,0.00),
          'black': (20.00,0.00,0.00),
          'purple': (39.21,74.61,-95.66)}


def colorDistance(left, right):
    return sum((l-r)**2 for l, r in zip(left, right))**0.5

class NearestColorKey(object):
    def __init__(self, goal):
        self.goal = goal
    def __call__(self, item):
        return colorDistance(self.goal, item[1])


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
        transformedPoints.append([transformedPoint.point.x,transformedPoint.point.y,0])
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

def tooClose():
    global transformedPoints
    global attackedHumans

    humanGoals = []

    for point in transformedPoints:
        if point[2] == 3:
            humanGoals.insert(0,point)

    for i, humanGoal in enumerate(humanGoals):
        for other in humanGoals[i+1:]:
            distance = findDistance(humanGoal, other)

            attack = True

            for i in attackedHumans:
                    if i[0] == humanGoal[0] and i[1] == humanGoal[1] or i[0] == other[0] and i[1] == other[1]:
                        attack = False

            if (distance < 2) & attack:
                return [humanGoal, other]
    
    return None
    

def getHalfwayPoint(start_point, end_point):
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
    path = list(map(lambda point : [point.pose.position.x,point.pose.position.y], path.plan.poses))
    return path[int(len(path) / 2)]
    

def navigate():
    global transformedPoints
    global cylinderOrientation
    global faceOrientation
    global attackedHumans
    global faceData
    global detected_speech

    soundhandle = SoundClient()

    pub_arm = rospy.Publisher('arm_command', String)
    odom: Odometry = rospy.wait_for_message("/odom", Odometry)
    action_client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    action_client.wait_for_server()

    position = [odom.pose.pose.position.x, odom.pose.pose.position.y, 0]
    transformedPoints = np.insert(transformedPoints,0,[position],axis=0)
    transformedPoints = transformedPoints[findShortestPath(transformedPoints)]
    transformedPoints = transformedPoints[1:,:]
    print(transformedPoints)
    
    #rate = rospy.Rate(1)
    #while not rospy.is_shutdown():
    #    drawMarkers(transformedPoints)
    #    rate.sleep()
    
    while transformedPoints.size > 0:

        humansTooClose = tooClose() 

        if humansTooClose != None:
            halfwayPoint = getHalfwayPoint(humansTooClose[0], humansTooClose[1])
            halfwayPoint.append(4)
            transformedPoints = np.insert(transformedPoints, 0, halfwayPoint, axis=0)
            attackedHumans.insert(0, humansTooClose[0])
            attackedHumans.insert(0, humansTooClose[1])

        goal_point = transformedPoints[0,:]
        transformedPoints = transformedPoints[1:,:]
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"

        if goal_point[2] == 4:
            goal.target_pose.pose.position.x = goal_point[0]
            goal.target_pose.pose.position.y = goal_point[1]
            goal.target_pose.pose.orientation.z = faceOrientation[0, 0]
            goal.target_pose.pose.orientation.w = faceOrientation[0, 1]
            goal.target_pose.header.stamp = rospy.Time.now()

            action_client.send_goal(goal)
            while action_client.get_state() in [0,1]:
                time.sleep(1)
            
            soundhandle.say("Danger! Please move further apart.")

        elif goal_point[2] == 3:
            face_x = goal_point[0]
            face_y = goal_point[1]
            face_z = faceOrientation[0, 0]
            face_w = faceOrientation[0, 1]
            faceOrientation = faceOrientation[1:, :]

            goal.target_pose.pose.position.x = face_x
            goal.target_pose.pose.position.y = face_y
            goal.target_pose.pose.orientation.z = face_z
            goal.target_pose.pose.orientation.w = face_w
            goal.target_pose.header.stamp = rospy.Time.now()
            
            action_client.send_goal(goal)
            while action_client.get_state() in [0,1]:
                time.sleep(1)

            s = np.arcsin(face_z)
            c = np.arccos(face_w)
            angle = np.sign(s)*np.sign(c)*np.abs(s)
            new_face_z = np.sin(angle + np.pi/12)
            new_face_w = np.cos(angle + np.pi/12)

            goal.target_pose.pose.position.x = face_x
            goal.target_pose.pose.position.y = face_y
            goal.target_pose.pose.orientation.z = new_face_z
            goal.target_pose.pose.orientation.w = new_face_w
            goal.target_pose.header.stamp = rospy.Time.now()
            
            action_client.send_goal(goal)
            while action_client.get_state() in [0,1]:
                time.sleep(1)

            for i in faceData:
                if (i["x"] == goal_point[0]) & (i["y"] == goal_point[1]):
                    if i["mask"] == False:
                        soundhandle.say("Put on mask.")

                    soundhandle.say("Have you been vaccinated? Who is your Doctor? How many hours per week do you Exercise?")

                    while detected_speech == "":
                        time.sleep(1)

                    vaccinated = detected_speech.data.split(" ")[0]
                    doctor_name = detected_speech.data.split(" ")[1]
                    hours_of_exercise = detected_speech.data.split(" ")[2]

                    i["vaccinated"] = vaccinated
                    i["doctor"] = doctor_name
                    i["exercise"] = hours_of_exercise

                    detected_speech = ""


            

        elif goal_point[2] == 2:
            c = classifyColor()
            goal.target_pose.pose.position.x = goal_point[0]
            goal.target_pose.pose.position.y = goal_point[1]
            goal.target_pose.pose.orientation.z = cylinderOrientation[0, 0]
            goal.target_pose.pose.orientation.w = cylinderOrientation[0, 1]
            goal.target_pose.header.stamp = rospy.Time.now()
            cylinderOrientation = cylinderOrientation[1:, :]

            action_client.send_goal(goal)
            while action_client.get_state() in [0,1]:
                time.sleep(1)

            soundhandle.say(c)

            pub_arm.publish("extend")
            time.sleep(3)
            pub_arm.publish("retract")

        elif goal_point[2] == 1:
            c = classifyColor()
            goal.target_pose.pose.position.x = goal_point[0]
            goal.target_pose.pose.position.y = goal_point[1]
            goal.target_pose.pose.orientation.z = 0.1
            goal.target_pose.pose.orientation.w = 0.1
            goal.target_pose.header.stamp = rospy.Time.now()

            action_client.send_goal(goal)
            while action_client.get_state() in [0,1]:
                time.sleep(1)

            soundhandle.say(c)

        else:
            goal.target_pose.pose.position.x = goal_point[0]
            goal.target_pose.pose.position.y = goal_point[1]
            goal.target_pose.pose.orientation.z = 1
            goal.target_pose.pose.orientation.w = 0
            # goal.target_pose.pose.orientation.z = 0.95
            # goal.target_pose.pose.orientation.w = 0.29
            goal.target_pose.header.stamp = rospy.Time.now()

            action_client.send_goal(goal)
            while action_client.get_state() in [0,1]:
                time.sleep(1)

            goal.target_pose.pose.orientation.z = 0
            goal.target_pose.pose.orientation.w = 1
            # goal.target_pose.pose.orientation.z = -0.32
            # goal.target_pose.pose.orientation.w = 0.95
            goal.target_pose.header.stamp = rospy.Time.now()

            action_client.send_goal(goal)
            while action_client.get_state() in [0,1]:
                time.sleep(1)

        # transformedPoints = transformedPoints[findShortestPath(transformedPoints)]


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


def classifyColor():
    global markerColor
    rgb = [markerColor[0].r*255, markerColor[0].g*255, markerColor[0].b*255]
    cylinderColor = rgb2lab(rgb)
    print(cylinderColor)
    color = min(colors.items(), key=NearestColorKey(cylinderColor))
    print(color)
    markerColor = markerColor[1:]
    return color[0]


def ringMarkersCallback(ring_markers: MarkerArray):
    global ring_num
    global transformedPoints
    global markerColor
    if len(ring_markers.markers) > ring_num:
        ring_goals = ring_markers.markers[ring_num:]
        for ring_goal in ring_goals:
            transformedPoints = np.insert(transformedPoints, 0, [ring_goal.pose.position.x, ring_goal.pose.position.y, 1], axis=0)
            markerColor = [ring_goal.color] + markerColor
        ring_num=len(ring_markers.markers)


def cylinderMarkersCallback(cylinder_markers: MarkerArray):
    global cylinder_num
    global transformedPoints
    global cylinderOrientation
    global markerColor
    if len(cylinder_markers.markers) > cylinder_num:
        cylinder_goals = cylinder_markers.markers[cylinder_num:]
        for cylinder_goal in cylinder_goals:
            transformedPoints = np.insert(transformedPoints, 0, [cylinder_goal.pose.position.x, cylinder_goal.pose.position.y, 2], axis=0)
            cylinderOrientation = np.insert(cylinderOrientation, 0, [cylinder_goal.pose.orientation.z, cylinder_goal.pose.orientation.w], axis=0)
            markerColor = [cylinder_goal.color] + markerColor
        cylinder_num = len(cylinder_markers.markers)

def faceGoalsCallback(face_goals_array: FaceGoalsArray):
    global face_goals_num
    global transformedPoints
    global faceOrientation
    global faceData

    if len(face_goals_array.goals) > face_goals_num:
        face_goals = face_goals_array.goals[face_goals_num:]
        for face_goal in face_goals:
            transformedPoints = np.insert(transformedPoints, 0, [face_goal.coords[0],face_goal.coords[1],3], axis=0)
            faceOrientation = np.insert(faceOrientation, 0, [face_goal.coords[2],face_goal.coords[3]], axis=0)
            faceData.insert(0, {'x':face_goal.coords[0], 'y':face_goal.coords[1], 'mask':face_goal.wearing_mask, 'exercise':0, 'age':0, 'doctor':"", 'vaccine':"", "vaccinated":""})
        face_goals_num = len(face_goals_array.goals)

def detectedSpeechCallback(detectedSpeech: String):
    global detected_speech
    detected_speech = detectedSpeech

def rgb2lab(inputColor):
    num = 0
    RGB = [0, 0, 0]

    for value in inputColor:
        value = float(value) / 255

        if value > 0.04045:
            value = ((value + 0.055) / 1.055) ** 2.4
        else:
            value = value / 12.92

        RGB[num] = value * 100
        num = num + 1

    XYZ = [0, 0, 0, ]

    X = RGB[0] * 0.4124 + RGB[1] * 0.3576 + RGB[2] * 0.1805
    Y = RGB[0] * 0.2126 + RGB[1] * 0.7152 + RGB[2] * 0.0722
    Z = RGB[0] * 0.0193 + RGB[1] * 0.1192 + RGB[2] * 0.9505
    XYZ[0] = round(X, 4)
    XYZ[1] = round(Y, 4)
    XYZ[2] = round(Z, 4)

    # Observer= 2°, Illuminant= D65
    XYZ[0] = float(XYZ[0]) / 95.047         # ref_X =  95.047
    XYZ[1] = float(XYZ[1]) / 100.0          # ref_Y = 100.000
    XYZ[2] = float(XYZ[2]) / 108.883        # ref_Z = 108.883

    num = 0
    for value in XYZ:

        if value > 0.008856:
            value = value ** (0.3333333333333333)
        else:
            value = (7.787 * value) + (16 / 116)

        XYZ[num] = value
        num = num + 1

    Lab = [0, 0, 0]

    L = (116 * XYZ[1]) - 16
    a = 500 * (XYZ[0] - XYZ[1])
    b = 200 * (XYZ[1] - XYZ[2])

    Lab[0] = round(L, 4)
    Lab[1] = round(a, 4)
    Lab[2] = round(b, 4)

    return Lab


if __name__ == "__main__":
    rospy.init_node("autonomous_navigation")
    ros_map = rospy.wait_for_message("map",OccupancyGrid)

    centres: np.array = getPoints(ros_map)
    transformedPoints = transformCoordinates(ros_map, centres)
    cylinderOrientation = np.empty(shape=(0,2))
    faceOrientation = np.empty(shape=(0,2))
    faceData = []
    markerColor = []
    detected_speech = ""
    rospy.Subscriber("cylinder_offsets", MarkerArray, cylinderMarkersCallback)
    rospy.Subscriber("face_goals",FaceGoalsArray, faceGoalsCallback)
    rospy.Subscriber("detected_speech", String, detectedSpeechCallback)
    navigate()

    #rate = rospy.Rate(1)
    #while not rospy.is_shutdown():
    #    drawMarkers(transformedPoints[np.argsort(distances)])
    #    rate.sleep()