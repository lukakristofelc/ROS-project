import rospy
import actionlib
import move_base_msgs.msg
import sys
import time
from homework4.msg import FaceGoalsArray,FaceGoals
from sound_play.libsoundplay import SoundClient

goalList = [{"goal_type":"main","num":1,"cords":[-0.9941200017929077,-0.005554626230150461,1,0]},
{"goal_type":"main","num":2,"cords":[-1.0450513362884521,1.8614863157272339,-0.9457620855911835,0.3248600890509868]},
{"goal_type":"main","num":3,"cords":[0.9646940231323242,2.0902199745178223,-0.29305322775315973,0.9560961278571599]},
{"goal_type":"main","num":4,"cords":[2.31929349899292,1.3784891366958618,-0.6528318346730345,0.7575028684021201]},
{"goal_type":"main","num":5,"cords":[1.2057281732559204,1.0228265523910522,0.9940693162261288,0.1087483082062273]},
{"goal_type":"main","num":6,"cords":[1.2110233306884766,0.3865589201450348,-0.8755446259684289,0.48313725579570455]},
{"goal_type":"main","num":7,"cords":[2.816470146179199,-0.49132999777793884,-0.0793839503721715,0.996844114404709]},
{"goal_type":"main","num":8,"cords":[-0.0091780424118042,-1.3134379386901855,-0.7879682852167199,0.6157158285220723]},
{"goal_type":"main","num":9,"cords":[-0.0067095333547065195,0.00041933767553622504,0.00022929397050545613,0.9999999737121372]}]

def callback_active():
    pass
    #rospy.loginfo("Robot is trying to reach next goal")

def callback_feedback(feedback:move_base_msgs.msg.MoveBaseFeedback):
    #rospy.loginfo("Robot is on the move towards the goal at position x: {} y: {}".format(feedback.base_position.pose.position.x,feedback.base_position.pose.position.y))
    pass

def callback_done(state, result):
    if state == 3:
        rospy.loginfo("Robot has reached the goal successfully")
    elif state == 2:
        rospy.loginfo("Goal has been aborted probably because of face")
    else:
        rospy.logwarn("Robot couldn't reach the goal {}".format(state))

def goToGoal():
    client = actionlib.SimpleActionClient('move_base',move_base_msgs.msg.MoveBaseAction)
    client.wait_for_server()
    soundhandle = SoundClient()

    face_goal_num=0
    #print(goalList)
    while len(goalList)>0:
        g = goalList.pop(0)
        goal = move_base_msgs.msg.MoveBaseGoal()
        rospy.loginfo("Preparing for {} goal {}".format(g["goal_type"],"num {}".format(g["num"]) if g["goal_type"]=="main" else ""))
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = g["cords"][0]
        goal.target_pose.pose.position.y = g["cords"][1]
        goal.target_pose.pose.orientation.z = g["cords"][2]
        goal.target_pose.pose.orientation.w = g["cords"][3]
        goal.target_pose.header.stamp = rospy.Time.now()
        client.send_goal(goal,active_cb=callback_active,done_cb=callback_done,feedback_cb=callback_feedback)
        while client.get_state() in [0,1]:
            face_goal_message:FaceGoalsArray = rospy.wait_for_message("face_goals",FaceGoalsArray)
            if len(face_goal_message.goals)>face_goal_num:
                if not g["goal_type"]=="face":
                    client.cancel_goal()
                    goalList.insert(0,g)
                face_goals = face_goal_message.goals[face_goal_num:]
                for face_goal in face_goals:
                    goalList.insert(0,{"goal_type":"face","cords":face_goal.coords})
                face_goal_num=len(face_goal_message.goals)
            time.sleep(1)
        #if client.get_state() == 3 and g["goal_type"]=="face":
        #    soundhandle.say("Hello")   


if __name__ == "__main__":
    if(len(sys.argv)==2 and sys.argv[1]=="restart"): 
        goalList = [{"goal_type":"main","num":1,"cords":[-0.0067095333547065195,0.00041933767553622504,0.00022929397050545613,0.9999999737121372]}]
    elif(len(sys.argv)==2 and sys.argv[1]=="lost"):
        goalList = [[0.16398291289806366,1.552408218383789,0,1]]
    rospy.init_node("hardcoded_goals")
    goToGoal()