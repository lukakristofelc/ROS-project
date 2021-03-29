import rospy
import actionlib
import move_base_msgs.msg
import sys

goalList = [[-0.9941200017929077,-0.005554626230150461,1,0],
[-1.0450513362884521,1.8614863157272339,-0.9457620855911835,0.3248600890509868],
[0.9646940231323242,2.0902199745178223,-0.29305322775315973,0.9560961278571599],
[2.31929349899292,1.3784891366958618,-0.6528318346730345,0.7575028684021201],
[1.2057281732559204,1.0228265523910522,0.9940693162261288,0.1087483082062273],
[1.2110233306884766,0.3865589201450348,-0.8755446259684289,0.48313725579570455],
[2.816470146179199,-0.49132999777793884,-0.0793839503721715,0.996844114404709],
[-0.0091780424118042,-1.3134379386901855,-0.7879682852167199,0.6157158285220723],
[-0.0067095333547065195,0.00041933767553622504,0.00022929397050545613,0.9999999737121372]]

def callback_active():
    rospy.loginfo("Robot is trying to reach next goal")

def callback_feedback(feedback:move_base_msgs.msg.MoveBaseFeedback):
    rospy.loginfo("\rRobot is on the move towards the goal at position x: {} y: {}".format(feedback.base_position.pose.position.x,feedback.base_position.pose.position.y))

def callback_done(state, result):
    if(state == 3):
        rospy.loginfo("Robot has reached the goal successfully")
    else:
        rospy.logwarn("Robot couldn't reach the goal")

def goToGoal():
    client = actionlib.SimpleActionClient('move_base',move_base_msgs.msg.MoveBaseAction)
    client.wait_for_server()

    print(goalList)
    for i,g in enumerate(goalList):
        goal = move_base_msgs.msg.MoveBaseGoal()
        rospy.loginfo("Preparing goal {}".format(i))
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = g[0]
        goal.target_pose.pose.position.y = g[1]
        goal.target_pose.pose.orientation.z = g[2]
        goal.target_pose.pose.orientation.w = g[3]
        goal.target_pose.header.stamp = rospy.Time.now()
        client.send_goal(goal,active_cb=callback_active,done_cb=callback_done,feedback_cb=callback_feedback)
        client.wait_for_result()

if __name__ == "__main__":
    if(len(sys.argv)==2 and sys.argv[1]=="restart"): 
        goalList = [[-0.0067095333547065195,0.00041933767553622504,0.00022929397050545613,0.9999999737121372]]
    elif(len(sys.argv)==2 and sys.argv[1]=="lost"):
        goalList = [[0.16398291289806366,1.552408218383789,0,1]]
    rospy.init_node("hardcoded_goals")
    goToGoal()