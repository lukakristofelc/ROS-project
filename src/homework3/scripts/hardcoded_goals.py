import rospy
import actionlib
import move_base_msgs.msg
import sys

goalList = [[-0.7588241100311279,1.9050352573394775,0.36879214999665577,0.9295118880901116],[1.0288082361221313,1.54348886013031,-0.4682524440955771,0.8835947309703236],[2.3728225231170654,-0.5523499846458435,-0.8831132281574203,0.46915991543756164],[0.008767127990722656,-1.300948143005371,0.7061932646803702,0.7080191190356943],[-0.0067095333547065195,0.00041933767553622504,0.00022929397050545613,0.9999999737121372]]

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