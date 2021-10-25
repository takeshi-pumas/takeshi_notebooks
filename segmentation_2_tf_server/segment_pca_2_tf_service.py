#! /usr/bin/env python
import rospy                                      # the main module for ROS-python programs
import sys
from std_srvs.srv import Trigger, TriggerResponse # we are creating a 'Trigger service'...
#from tmc_tabletop_segmentator.srv import TabletopSegmentation
#from tmc_tabletop_segmentator.srv import TabletopSegmentationRequest
from utils_server import *
sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages')
import cv2 

                                                  # ...Other types are available, and you can create
                                                  # custom types


class RGBD():


    def __init__(self):
        self._br = tf.TransformBroadcaster()
       
        self._cloud_sub = rospy.Subscriber(
            "/hsrb/head_rgbd_sensor/depth_registered/rectified_points",
            PointCloud2, self._cloud_cb)
        self._points_data = None
        self._image_data = None
        self._h_image = None
        self._region = None
        self._h_min = 0
        self._h_max = 0
        self._xyz = [0, 0, 0]
        self._frame_name = None

    def _cloud_cb(self, msg):
       
        self._points_data = ros_numpy.numpify(msg)

       
        self._image_data = \
            self._points_data['rgb'].view((np.uint8, 4))[..., [2, 1, 0]]

       
        hsv_image = cv2.cvtColor(self._image_data, cv2.COLOR_RGB2HSV_FULL)
        self._h_image = hsv_image[..., 0]

   
      

    def get_image(self):
        return self._image_data

    def get_points(self):
        return self._points_data



rgbd = RGBD()

def trigger_response(request):
    ''' 
    Callback function used by the service server to process
    requests from clients. It returns a TriggerResponse
    '''
###########################################################





#define a tabletop segmentation request.
# Play with these parameters

    listener = tf.TransformListener()
    broadcaster= tf.TransformBroadcaster()
    tf_static_broadcaster= tf2_ros.StaticTransformBroadcaster()
    #bridge = CvBridge()



    
    image= rgbd.get_image()
    points_data= rgbd.get_points()
    print (image.shape)
    values=image.reshape((-1,3))
    values= np.float32(values)
    criteria= (  cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER  ,1000,0.1)
    k=6
    _ , labels , cc =cv2.kmeans(values , k ,None,criteria,30,cv2.KMEANS_RANDOM_CENTERS)
    cc=np.uint8(cc)
    segmented_image= cc[labels.flatten()]
    segmented_image=segmented_image.reshape(image.shape)
    th3 = cv2.adaptiveThreshold(segmented_image,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,2)
    kernel = np.ones((5,5),np.uint8)
    im4=cv2.erode(th3,kernel,iterations=4)
    plane_mask=points_data['z']
    cv2_img=plane_mask.astype('uint8')
    img=im4
    _,contours, hierarchy = cv2.findContours(im4.astype('uint8'),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    i=0
    cents=[]
    points=[]
    """    criteria= (  cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER  ,1000,0.1)
                    k=6
                    _ , labels , cc =cv2.kmeans(values , k ,None,criteria,30,cv2.KMEANS_RANDOM_CENTERS)
                    cc=np.uint8(cc)
                    segmented_image= cc[labels.flatten()]
                    segmented_image=segmented_image.reshape(image.shape)
                    th3 = cv2.adaptiveThreshold(segmented_image,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,2)
                    kernel = np.ones((5,5),np.uint8)
                    im4=cv2.erode(th3,kernel,iterations=4)
                    plane_mask=points_data['z']
                    cv2_img=plane_mask.astype('uint8')
                    img=im4
                    _,contours, hierarchy = cv2.findContours(im4.astype('uint8'),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
                    i=0
                    cents=[]
                    points=[]
                    for i, contour in enumerate(contours):
                        
                        area = cv2.contourArea(contour)
            
                        if area > lower and area < higher :
                            M = cv2.moments(contour)
                            # calculate x,y coordinate of center
                            cX = int(M["m10"] / M["m00"])
                            cY = int(M["m01"] / M["m00"])
                            
                    
                            boundRect = cv2.boundingRect(contour)
                            #just for drawing rect, dont waste too much time on this
            
                            img=cv2.rectangle(img,(boundRect[0], boundRect[1]),(boundRect[0]+boundRect[2], boundRect[1]+boundRect[3]), (0,0,0), 2)
                            # calculate moments for each contour
                            if (cY > reg_ly and cY < reg_hy  ):
                                
                                cv2.circle(img, (cX, cY), 5, (255, 255, 255), -1)
                                cv2.putText(img, "centroid_"+str(i)+"_"+str(cX)+','+str(cY)    ,    (cX - 25, cY - 25)   ,cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 2)
                                print ('cX,cY',cX,cY)
                                xyz=[]
            
            
                                for jy in range (boundRect[0], boundRect[0]+boundRect[2]):
                                    for ix in range(boundRect[1], boundRect[1]+boundRect[3]):
                                        aux=(np.asarray((points_data['x'][ix,jy],points_data['y'][ix,jy],points_data['z'][ix,jy])))
                                        if np.isnan(aux[0]) or np.isnan(aux[1]) or np.isnan(aux[2]):
                                            'reject point'
                                        else:
                                            xyz.append(aux)
            
                                xyz=np.asarray(xyz)
                                cent=xyz.mean(axis=0)
                                cents.append(cent)
                                print (cent)
                                points.append(xyz)
                            else:
                                print ('cent out of region... rejected')
                            
                    cents=np.asarray(cents)
                    ### returns centroids found and a group of 3d coordinates that conform the centroid
                    return(cents,np.asarray(points))





    objs_depth_centroids= sort_list (objs_depth_centroids)
    for i in range(len(objs_depth_centroids)):
        #Table is a plane at z=.8 So we consider false positives all the centroids outside the region on axis z ( .79 , .9)
        if objs_depth_centroids[i][2] > 0 and objs_depth_centroids[i][2] <44.9: 
            static_transformStamped = geometry_msgs.msg.TransformStamped()
            broadcaster.sendTransform((objs_depth_centroids[i]),(0,0,0,1), rospy.Time.now(), 'Object'+str(i),"head_rgbd_sensor_link")

    """
    return TriggerResponse(
        success=True,
        message="Segmentation requested"
    )

rospy.init_node('segment_2_tf_service')                     # initialize a ROS node
rgbd = RGBD()
#image= rgbd.get_image()
points_data= rgbd.get_points()
segment_pca = rospy.Service(                        # create a service, specifying its name,
    '/segment_2_tf', Trigger, trigger_response         # type, and callback
)
rospy.loginfo("segmentation service available")
rospy.spin()   
