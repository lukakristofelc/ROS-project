import rospy
import actionlib
import move_base_msgs.msg
import sys
import time
from homework4.msg import FaceGoalsArray,FaceGoals
from sound_play.libsoundplay import SoundClient


# goalList = [{"goal_type":"main","num":1,"cords":[-0.9941200017929077,-0.005554626230150461,1,0]},
# {"goal_type":"main","num":2,"cords":[-1.0450513362884521,1.8614863157272339,-0.9457620855911835,0.3248600890509868]},
# {"goal_type":"main","num":3,"cords":[0.9646940231323242,2.0902199745178223,-0.29305322775315973,0.9560961278571599]},
# {"goal_type":"main","num":4,"cords":[2.31929349899292,1.3784891366958618,-0.6528318346730345,0.7575028684021201]},
# {"goal_type":"main","num":5,"cords":[1.2057281732559204,1.0228265523910522,0.9940693162261288,0.1087483082062273]},
# {"goal_type":"main","num":6,"cords":[1.2110233306884766,0.3865589201450348,-0.8755446259684289,0.48313725579570455]},
# {"goal_type":"main","num":7,"cords":[2.816470146179199,-0.49132999777793884,-0.0793839503721715,0.996844114404709]},
# {"goal_type":"main","num":8,"cords":[-0.0091780424118042,-1.3134379386901855,-0.7879682852167199,0.6157158285220723]},
# {"goal_type":"main","num":9,"cords":[-0.0067095333547065195,0.00041933767553622504,0.00022929397050545613,0.9999999737121372]}]

# goalList = [{"goal_type":"main","num":1,"cords":[-1.339717149734497,0.07583579421043396,-0.8782452195281119,0.4782105544381247]},
# {"goal_type":"main","num":2,"cords":[-0.759399950504303,0.777918815612793,0.9995968447043939,0.02839274655646687]},
# {"goal_type":"main","num":3,"cords":[-1.2742784023284912,1.8375455141067505,-0.9439855828820547,0.329986695657306]},
# {"goal_type":"main","num":4,"cords":[-0.19575892388820648,2.512491226196289,0.8998179756142902,0.43626552781694844]},
# {"goal_type":"main","num":5,"cords":[0.10576514899730682,2.3561675548553467,-0.7095189003375146,0.704686405476822]},
# {"goal_type":"main","num":6,"cords":[0.9881874322891235,2.1187424659729004,-0.2702254275780399,0.9627970805368417]},
# {"goal_type":"main","num":7,"cords":[2.4423182010650635,2.191869020462036,-0.00341678956665327,0.9999941627574919]},
# {"goal_type":"main","num":8,"cords":[2.402463436126709,1.5691568851470947,-0.9999957611699737,0.0029116390718998135]},
# {"goal_type":"main","num":9,"cords":[1.1940685510635376,0.99003005027771,0.9681433538470967,0.2503965782547663]},
# {"goal_type":"main","num":10,"cords":[1.1186972856521606,0.37942683696746826,-0.709518564316771,0.704686743801718]},
# {"goal_type":"main","num":11,"cords":[2.1456987857818604,0.46140575408935547,0.03223117918018036,0.9994804405733286]},
# {"goal_type":"main","num":12,"cords":[2.3652734756469727,-0.3922959566116333,0.1019516687065269,0.9947893532039608]},
# {"goal_type":"main","num":13,"cords":[1.0241215229034424,-1.3049519062042236,0.704686563362728,0.7095187435269261]},
# {"goal_type":"main","num":14,"cords":[0.027555078268051147,-1.2744081020355225,-0.7095189003375146,0.704686405476822]},
# {"goal_type":"main","num":15,"cords":[-0.0067095333547065195,0.00041933767553622504,0.00022929397050545613,0.9999999737121372]}]

goalList = [{"goal_type":"main","num":1,"cords":[-0.010029196739196777,-1.245214581489563,0.9999941613443316,0.003417203132233015]},
{"goal_type":"main","num":2,"cords":[1.1483616828918457,-1.315213680267334,0.7743763834748276,0.6327252300299449]},
{"goal_type":"main","num":3,"cords":[2.6747512817382812,-0.5289226770401001,-0.3522253795764856,0.9359152108937009]},
{"goal_type":"main","num":4,"cords":[1.034668207168579,0.21692630648612976,-0.709518648321972,0.704686659220509]},
{"goal_type":"main","num":5,"cords":[2.1255412101745605,0.8943219780921936,0.008775359078526309,0.9999614957952345]},
{"goal_type":"main","num":6,"cords":[2.445707082748413,2.0904626846313477,-0.011228733345816424,0.9999369557864378]},
{"goal_type":"main","num":7,"cords":[1.0771678686141968,2.0856335163116455,-0.925181817798718,0.37952418106710895]},
{"goal_type":"main","num":8,"cords":[0.5901126861572266,2.40804386138916,0.7102075294845817,0.7039923757139754]},
{"goal_type":"main","num":9,"cords":[-1.4119892120361328,2.038827896118164,0.8218107752423282,0.5697605196006509]},
{"goal_type":"main","num":10,"cords":[-0.9887474775314331,1.7168538570404053,-0.9898908437783352,0.1418312990979614]},
{"goal_type":"main","num":11,"cords":[-0.7419974207878113,0.8320295810699463,0.022872744191338227,0.999738384565261]},
{"goal_type":"main","num":12,"cords":[-1.0790959596633911,0.1109398752450943,0.9997738808175242,0.021264694567922286]},
{"goal_type":"main","num":13,"cords":[-0.1126602441072464,0.03051925078034401,-0.011480720880721123,0.9999340943522523]}]

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
                face_goals = face_goal_message.goals[face_goal_num:]
                for face_goal in face_goals:
                    goalList.insert(0,{"goal_type":"face","cords":face_goal.coords})
                face_goal_num=len(face_goal_message.goals)
            time.sleep(1)
        if client.get_state() == 3 and g["goal_type"]=="face":
            soundhandle.say("Hello")


if __name__ == "__main__":
    if(len(sys.argv)==2 and sys.argv[1]=="restart"): 
        goalList = [{"goal_type":"main","num":1,"cords":[-0.0067095333547065195,0.00041933767553622504,0.00022929397050545613,0.9999999737121372]}]
    elif(len(sys.argv)==2 and sys.argv[1]=="lost"):
        goalList = [[0.16398291289806366,1.552408218383789,0,1]]
    rospy.init_node("hardcoded_goals")
    goToGoal()