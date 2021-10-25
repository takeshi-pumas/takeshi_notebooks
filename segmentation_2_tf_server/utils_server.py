# -*- coding: utf-8 -*-
from cv_bridge import CvBridge, CvBridgeError
import rospy
import cv2
import ros_numpy
import numpy as np

from sensor_msgs.msg import Image , LaserScan, PointCloud2
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Quaternion ,TransformStamped
import tf
import tf2_ros
import geometry_msgs.msg


def static_tf_publish(cents):
    trans , rot = listener.lookupTransform('/map', '/head_rgbd_sensor_gazebo_frame', rospy.Time(0))
    for  i ,cent  in enumerate(cents):
        x,y,z=cent
        if np.isnan(x) or np.isnan(y) or np.isnan(z):
            print('nan')
        else:
            broadcaster.sendTransform((x,y,z),rot, rospy.Time.now(), 'Object'+str(i),"head_rgbd_sensor_link")
            rospy.sleep(.2)
            xyz_map,cent_quat= listener.lookupTransform('/map', 'Object'+str(i),rospy.Time(0))
            map_euler=tf.transformations.euler_from_quaternion(cent_quat)
            rospy.sleep(.2)
            static_transformStamped = TransformStamped()
            

            ##FIXING TF TO MAP ( ODOM REALLY)    
            #tf_broadcaster1.sendTransform( (xyz[0],xyz[1],xyz[2]),tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "obj"+str(ind), "head_rgbd_sensor_link")
            static_transformStamped.header.stamp = rospy.Time.now()
            static_transformStamped.header.frame_id = "map"
            
            static_transformStamped.transform.translation.x = float(xyz_map[0])
            static_transformStamped.transform.translation.y = float(xyz_map[1])
            static_transformStamped.transform.translation.z = float(xyz_map[2])
            #quat = tf.transformations.quaternion_from_euler(-euler[0],0,1.5)
            static_transformStamped.transform.rotation.x = 0#-quat[0]#trans.transform.rotation.x
            static_transformStamped.transform.rotation.y = 0#-quat[1]#trans.transform.rotation.y
            static_transformStamped.transform.rotation.z = 0#-quat[2]#trans.transform.rotation.z
            static_transformStamped.transform.rotation.w = 1#-quat[3]#trans.transform.rotation.w

            if xyz_map[2] > .4 and xyz_map[2] < .46:   #table 1 
                static_transformStamped.child_frame_id = "Object_"+str(i)+"_Table_1"
                tf_static_broadcaster.sendTransform(static_transformStamped)
                print xyz_map[2]
            if  xyz_map[2] < .15:   #table 1 
                static_transformStamped.child_frame_id = "Object_"+str(i)+"_Floor"
                tf_static_broadcaster.sendTransform(static_transformStamped)
                print xyz_map[2]
    return True
    
def  sort_list(centroids_list):
    cents= np.asarray(centroids_list)
    
    cents=cents[cents[:,2].argsort()]
    return cents.tolist()
    

    
def pose_2_np(wp_p):
   
    return np.asarray((wp_p.pose.position.x,wp_p.pose.position.y,wp_p.pose.position.z)) , np.asarray((wp_p.pose.orientation.w,wp_p.pose.orientation.x,wp_p.pose.orientation.y, wp_p.pose.orientation.z)) 
def np_2_pose(position,orientation):
    wb_p= geometry_msgs.msg.PoseStamped()
    
    wb_p.pose.position.x= position[0]
    wb_p.pose.position.y= position[1]
    wb_p.pose.position.z= position[2]
    wb_p.pose.orientation.w= orientation[0]
    wb_p.pose.orientation.x= orientation[1]
    wb_p.pose.orientation.y= orientation[2]
    wb_p.pose.orientation.z= orientation[3]
    return wb_p


def rot_to_euler(R):
    import sys
    tol = sys.float_info.epsilon * 10

    if abs(R.item(0,0))< tol and abs(R.item(1,0)) < tol:
       eul1 = 0
       eul2 = m.atan2(-R.item(2,0), R.item(0,0))
       eul3 = m.atan2(-R.item(1,2), R.item(1,1))
    else:   
       eul1 = m.atan2(R.item(1,0),R.item(0,0))
       sp = m.sin(eul1)
       cp = m.cos(eul1)
       eul2 = m.atan2(-R.item(2,0),cp*R.item(0,0)+sp*R.item(1,0))
       eul3 = m.atan2(sp*R.item(0,2)-cp*R.item(1,2),cp*R.item(1,1)-sp*R.item(0,1))

    return np.asarray((eul1,eul2,eul3))
def pca_xyz(xyz):
    quats=[]
    for i in range( len(xyz)):
        pca= PCA(n_components=3).fit(xyz[i])
        vec0= pca.components_[0,:]
        vec1= pca.components_[1,:]
        vec2= pca.components_[2,:]
        R=pca.components_
        euler=rot_to_euler(R)
        quats.append(tf.transformations.quaternion_from_euler(euler[0],euler[1],euler[2]))
    return quats

class RGBD():
    u"""RGB-Dデータを扱うクラス"""

    def __init__(self):
        self._br = tf.TransformBroadcaster()
        # ポイントクラウドのサブスクライバのコールバックに_cloud_cbメソッドを登録
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
        # ポイントクラウドを取得する
        self._points_data = ros_numpy.numpify(msg)

        # 画像を取得する
        self._image_data = \
            self._points_data['rgb'].view((np.uint8, 4))[..., [2, 1, 0]]

        # 色相画像を作成する
        hsv_image = cv2.cvtColor(self._image_data, cv2.COLOR_RGB2HSV_FULL)
        self._h_image = hsv_image[..., 0]

        # 色相の閾値内の領域を抽出する
        self._region = \
            (self._h_image > self._h_min) & (self._h_image < self._h_max)

        # 領域がなければ処理を終える
        if not np.any(self._region):
            return

        # 領域からxyzを計算する
        (y_idx, x_idx) = np.where(self._region)
        x = np.average(self._points_data['x'][y_idx, x_idx])
        y = np.average(self._points_data['y'][y_idx, x_idx])
        z = np.average(self._points_data['z'][y_idx, x_idx])
        self._xyz = [y, x, z]
        #self._xyz = [x, y, z]

        # 座標の名前が設定されてなければ処理を終える
        if self._frame_name is None:
            return

        # tfを出力する
        self._br.sendTransform(
            (x, y, z), tf.transformations.quaternion_from_euler(0, 0, 0),
            rospy.Time(msg.header.stamp.secs, msg.header.stamp.nsecs),
            self._frame_name,
            msg.header.frame_id)

    def get_image(self):
        u"""画像を取得する関数"""
        return self._image_data

    def get_points(self):
        u"""ポイントクラウドを取得する関数"""
        return self._points_data

    def get_h_image(self):
        u"""色相画像を取得する関数"""
        return self._h_image

    def get_region(self):
        u"""抽出領域の画像を取得する関数"""
        return self._region

    def get_xyz(self):
        u"""抽出領域から計算されたxyzを取得する関数"""
        return self._xyz

    def set_h(self, h_min, h_max):
        u"""色相の閾値を設定する関数"""
        self._h_min = h_min
        self._h_max = h_max

    def set_coordinate_name(self, name):
        u"""座標の名前を設定する関数"""
        self._frame_name = name
