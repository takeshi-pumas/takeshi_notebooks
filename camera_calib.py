# -*- coding: utf-8 -*-
"""
Created on Sun Jan 19 15:17:33 2020

@author: oscar
"""

#!/usr/bin/env python


import cv2
from geometry_msgs.msg import Quaternion
import numpy as np
import rospy
import message_filters
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from tf.transformations import *
from tf.msg import tfMessage
from tf.transformations import euler_from_quaternion
from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
objpoints = []
imgpoints = []
objp = np.zeros((4*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:4].T.reshape(-1,2)



from std_msgs.msg import String
def shutdown(self):
 
        exit()

def callback(img_msg):
        #$print( "got image")
        global objpoints,imgpoints,objp
       
        
    ################################################################ DO THINGS HERE
        cv2_img = bridge.imgmsg_to_cv2(img_msg, "bgr8")
        print ('shape', cv2_img.shape)
        gray = cv2.cvtColor(cv2_img,cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, (7,4),None)
        if (len(objpoints) >19):
        	ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
        	print ('camera matrix',mtx)
#        	('camera matrix', array([[694.53584855,   0.        , 201.46433583],
#            						[  0.        , 983.48927432, 181.24713249],
#     								[  0.        ,   0.        ,   1.        ]]))
# 		[683.39887245,   0.        , 216.13604054],
#       [  0.        , 928.11068076, 178.90078819],
#       [  0.        ,   0.        ,   1.        ]]))

	   #[996.93330319,   0.        , 237.70378617],
       #[  0.        , 740.29456939, 193.5517939 ],
       #[  0.        ,   0.        ,   1.        ]]





        if ret == True:
        	print("WE HAVE CORNERS" ,len(objpoints))
	        #objpoints.append(objp)

	        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)

	        # Draw and display the corners
	        cv2_img = cv2.drawChessboardCorners(cv2_img, (11,8), corners2,ret)
        
        cv2.namedWindow("xtion_cam")
        #cv2.imshow("hand_cam", img2)
        cv2.imshow("xtion_cam", cv2_img)    
        k = cv2.waitKey(1)
        if k & 0xFF == ord('q'):
            # q key pressed so quit
            print("Quitting...")
            kill_node=True
            cv2.destroyAllWindows()
            exit()
    
        elif k & 0xFF == ord('c'):
            # c key pressed so capture frame to image file
            print("appending")
            objpoints.append(objp)
            imgpoints.append(corners2)          
            


    
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('hand_camera_listener', anonymous=True)
    rospy.on_shutdown(shutdown)

    #rospy.Subscriber("/hsrb/hand_camera/image_raw", Image, callback)
    rospy.Subscriber("/hsrb/head_rgbd_sensor/rgb/image_rect_color", Image, callback)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    


if __name__ == '__main__':
    listener()
        # If found, add object points, image points (after refining them)
