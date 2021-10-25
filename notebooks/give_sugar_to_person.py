#!/usr/bin/env python

import smach
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Point , Quaternion
from actionlib_msgs.msg import GoalStatus
import moveit_commander
import moveit_msgs.msg
import tf2_ros
from utils_notebooks import *
from utils_takeshi import *
########## Functions for takeshi states ##########
class Proto_state(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'])
        self.tries=0
    def execute(self,userdata):
        rospy.loginfo('State : PROTO_STATE')

        if self.tries==3:
            self.tries=0 
            return'tries'
        if succ:
            return 'succ'
        else:
            return 'failed'
        global trans_hand
        self.tries+=1
        if self.tries==3:
            self.tries=0 
            return'tries'







def segment_table2(chan):
    image_data=rgbd.get_image()
    points_data = rgbd.get_points()

    mask=np.zeros((image_data.shape))
    plane_mask=np.zeros((image_data.shape[0],image_data.shape[1]))

    plane_mask=image_data[:,:,chan]

    ret,thresh = cv2.threshold(image_data[:,:,2],240,255,200)
    plane_mask=points_data['z']
    cv2_img=plane_mask.astype('uint8')
    img=image_data[:,:,0]
    _,contours, hierarchy = cv2.findContours(thresh.astype('uint8'),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    i=0
    cents=[]
    for i, contour in enumerate(contours):
        area = cv2.contourArea(contour)

        if area > 200 and area < 50000 :
            print('contour',i,'area',area)

            boundRect = cv2.boundingRect(contour)
            #just for drawing rect, dont waste too much time on this
            print boundRect
            img=cv2.rectangle(img,(boundRect[0], boundRect[1]),(boundRect[0]+boundRect[2], boundRect[1]+boundRect[3]), (0,0,0), 2)
            # calculate moments for each contour
            xyz=[]


            for jy in range (boundRect[0], boundRect[0]+boundRect[2]):
                for ix in range(boundRect[1], boundRect[1]+boundRect[3]):
                    xyz.append(np.asarray((points_data['x'][ix,jy],points_data['y'][ix,jy],points_data['z'][ix,jy])))
            xyz=np.asarray(xyz)
            cent=xyz.mean(axis=0)
            cents.append(cent)
            M = cv2.moments(contour)
            # calculate x,y coordinate of center
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            cv2.circle(img, (cX, cY), 5, (255, 255, 255), -1)
            cv2.putText(img, "centroid_"+str(i)+"_"+str(cX)+','+str(cY)    ,    (cX - 25, cY - 25)   ,cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 2)
            print ('cX,cY',cX,cY)
    cents=np.asarray(cents)
    
    return (cents)
def segment_table():
    image_data=rgbd.get_image()
    points_data = rgbd.get_points()

    mask=np.zeros((image_data.shape))
    plane_mask=np.zeros((image_data.shape[0],image_data.shape[1]))

    plane_mask=image_data[:,:,1]

    ret,thresh = cv2.threshold(image_data[:,:,2],240,255,200)
    plane_mask=points_data['z']
    cv2_img=plane_mask.astype('uint8')
    img=image_data[:,:,0]
    _,contours, hierarchy = cv2.findContours(thresh.astype('uint8'),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    i=0
    cents=[]
    for i, contour in enumerate(contours):
        area = cv2.contourArea(contour)

        if area > 2000 and area < 50000 :
            print('contour',i,'area',area)

            boundRect = cv2.boundingRect(contour)
            #just for drawing rect, dont waste too much time on this
            print boundRect
            img=cv2.rectangle(img,(boundRect[0], boundRect[1]),(boundRect[0]+boundRect[2], boundRect[1]+boundRect[3]), (0,0,0), 2)
            # calculate moments for each contour
            xyz=[]


            for jy in range (boundRect[0], boundRect[0]+boundRect[2]):
                for ix in range(boundRect[1], boundRect[1]+boundRect[3]):
                    xyz.append(np.asarray((points_data['x'][ix,jy],points_data['y'][ix,jy],points_data['z'][ix,jy])))
            xyz=np.asarray(xyz)
            cent=xyz.mean(axis=0)
            cents.append(cent)
            M = cv2.moments(contour)
            # calculate x,y coordinate of center
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            cv2.circle(img, (cX, cY), 5, (255, 255, 255), -1)
            cv2.putText(img, "centroid_"+str(i)+"_"+str(cX)+','+str(cY)    ,    (cX - 25, cY - 25)   ,cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 2)
            print ('cX,cY',cX,cY)
    cents=np.asarray(cents)
    return (cents)


##### Define state INITIAL #####
#Estado inicial de takeshi, neutral
class Initial(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'],input_keys=['global_counter'])
        self.tries=0

        
    def execute(self,userdata):

        
        rospy.loginfo('STATE : robot neutral pose')
        print('Try',self.tries,'of 5 attepmpts') 
        self.tries+=1
        scene.remove_world_object()
        #Takeshi neutral
        arm.set_named_target('go')
        arm.go()
        head.set_named_target('neutral')
        succ = head.go()             
        if succ:
            return 'succ'
        else:
            return 'failed'




##### Define state SCAN_TABLE #####
class Goto_table(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'],input_keys=['counter_in'],output_keys=['counter_out'])
        self.tries=0
    def execute(self,userdata):
        self.tries+=1
        if self.tries==3:
            self.tries=0 
            return'tries'
        global cents, rot, trans
        
        userdata.counter_out=userdata.counter_in +1

        #goal_x , goal_y, goal_yaw = kl_table1
        #move_base_goal(goal_x+.25*self.tries, goal_y , goal_yaw)      
        

        goal_x = 0.1
        goal_y = 1.2
        goal_yaw = 1.57

        # fill ROS message
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position = Point(goal_x, goal_y, 0)
        quat = tf.transformations.quaternion_from_euler(0, 0, goal_yaw)
        pose.pose.orientation = Quaternion(*quat)

        goal = MoveBaseGoal()
        goal.target_pose = pose

        # send message to the action server
        navclient.send_goal(goal)

        # wait for the action server to complete the order
        navclient.wait_for_result(timeout=rospy.Duration(10))

        # print result of navigation
        action_state = navclient.get_state()
        print (action_state)



        if action_state == GoalStatus.SUCCEEDED:
            rospy.loginfo("Navigation Succeeded.")
            return 'succ'
        else:
            return'failed'
    


class Scan_table(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'],input_keys=['counter_in'],output_keys=['counter_out'])
        self.tries=0
    def execute(self,userdata):
        self.tries+=1
        if self.tries==3:
            self.tries=0 
            return'tries'
        global cents, rot, trans
        
        userdata.counter_out=userdata.counter_in +1




        head_val = head.get_current_joint_values()
        head_val[0] = np.deg2rad(0)
        head_val[1] = np.deg2rad(-45)        
        succ = head.go(head_val)
        rospy.sleep(.2)
        
        trans, rot = listener.lookupTransform('/map', '/head_rgbd_sensor_gazebo_frame', rospy.Time(0))
        euler = tf.transformations.euler_from_quaternion(rot)        
        cents = segment_table()
        if len (cents)==0:
            cents = segment_table2(2)
            
            
                                            
        if len (cents)==0:
            arm.set_named_target('go')
            arm.go()
            head.set_named_target('neutral')
            head.go()
            return 'failed'
        else:
            print ('tfs published (not static)')
            #static_tf_publish(cents)
            self.tries=0 
            return 'succ'
        
class Goto_person(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'],input_keys=['counter_in'],output_keys=['counter_out'])
        self.tries=0
    def execute(self,userdata):
        self.tries+=1
        if self.tries==3:
            self.tries=0 
            return'tries'
        goal_x = 0.6
        goal_y = 3.3
        goal_yaw = 2*1.57

        # fill ROS message
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position = Point(goal_x, goal_y, 0)
        quat = tf.transformations.quaternion_from_euler(0, 0, goal_yaw)
        pose.pose.orientation = Quaternion(*quat)

        goal = MoveBaseGoal()
        goal.target_pose = pose

        # send message to the action server
        navclient.send_goal(goal)

        # wait for the action server to complete the order
        navclient.wait_for_result(timeout=rospy.Duration(10))

        # print result of navigation
        action_state = navclient.get_state()
        print(action_state)
        rospy.loginfo (str(whole_body.get_current_joint_values()[:2]))

        if action_state == GoalStatus.SUCCEEDED:
            rospy.loginfo("Navigation Succeeded.")
            return 'succ'
        else:
            print(action_state)
            return'failed'

class Give_object(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'],input_keys=['counter_in'],output_keys=['counter_out'])
        self.tries=0
    def execute(self,userdata):
        self.tries+=1
        print ('yey')
        return 'succ'






        if self.tries==3:
            self.tries=0 
            return'tries'
        


#Initialize global variables and node
def init(node_name):
    global listener, broadcaster, tfBuffer, tf_static_broadcaster, scene, rgbd  , head,whole_body,arm  ,goal,navclient
    rospy.init_node(node_name)
    head = moveit_commander.MoveGroupCommander('head')
    whole_body=moveit_commander.MoveGroupCommander('whole_body_weighted')
    arm =  moveit_commander.MoveGroupCommander('arm')
    listener = tf.TransformListener()
    broadcaster = tf.TransformBroadcaster()
    tfBuffer = tf2_ros.Buffer()
    tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster()
    whole_body.set_workspace([-6.0, -6.0, 6.0, 6.0]) 
    scene = moveit_commander.PlanningSceneInterface()
    rgbd = RGBD()
    goal = MoveBaseGoal()
    navclient = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)
    #navclient.wait_for_server()

     ##FIXING TF TO MAP ( ODOM REALLY)    
    static_transformStamped = TransformStamped()

    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "map"
    static_transformStamped.child_frame_id = "Drawer_high" 
    static_transformStamped.transform.translation.x = 0.14
    static_transformStamped.transform.translation.y = -0.344
    static_transformStamped.transform.translation.z = 0.57
    static_transformStamped.transform.rotation.x = 0    
    static_transformStamped.transform.rotation.y = 0    
    static_transformStamped.transform.rotation.z = 0    
    static_transformStamped.transform.rotation.w = 1    

    tf_static_broadcaster.sendTransform(static_transformStamped)

    
    
  

#Entry point    
if __name__== '__main__':
    print("Takeshi STATE MACHINE...")
    init("takeshi_smach")
    sm = smach.StateMachine(outcomes = ['END'])     #State machine, final state "END"
    sm.userdata.sm_counter = 0

    with sm:
        #State machine for grasping on Floor
        smach.StateMachine.add("INITIAL",       Initial(),      transitions = {'failed':'INITIAL',      'succ':'GOTO_TABLE',    'tries':'INITIAL'}) 
        smach.StateMachine.add("GOTO_TABLE",    Goto_table(),   transitions = {'failed':'GOTO_TABLE',   'succ':'SCAN_TABLE',     'tries':'INITIAL'},remapping={'counter_in':'sm_counter','counter_out':'sm_counter'})
        smach.StateMachine.add("SCAN_TABLE",    Scan_table(),   transitions = {'failed':'SCAN_TABLE',   'succ':'GOTO_PERSON',     'tries':'INITIAL'},remapping={'counter_in':'sm_counter','counter_out':'sm_counter'})
        smach.StateMachine.add("GOTO_PERSON",    Goto_person(),   transitions = {'failed':'GOTO_PERSON',   'succ':'GIVE_OBJECT',     'tries':'INITIAL'},remapping={'counter_in':'sm_counter','counter_out':'sm_counter'})
        smach.StateMachine.add("GIVE_OBJECT",    Give_object(),   transitions = {'failed':'GIVE_OBJECT',   'succ':'END',     'tries':'INITIAL'},remapping={'counter_in':'sm_counter','counter_out':'sm_counter'})
        

        

      

    outcome = sm.execute()


    