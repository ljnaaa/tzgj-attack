#!/usr/bin/env python2
 # -*- coding: UTF-8 -*- 
import rospy
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import sys
import os
import numpy as np
import pyrealsense2 as rs2
from realsense2_camera.msg import four_point
from realsense2_camera.msg import int_pub
from realsense2_camera.msg import AprilTagDetection
from realsense2_camera.msg import AprilTagDetectionArray
import cv2
import random
if (not hasattr(rs2, 'intrinsics')):
    import pyrealsense2.pyrealsense2 as rs2



class ImageListener:
    def __init__(self, depth_image_topic, depth_info_topic,point_info_topic,tf_transform):
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(depth_image_topic, msg_Image, self.imageDepthCallback)
        self.sub_info = rospy.Subscriber(depth_info_topic, CameraInfo, self.imageDepthInfoCallback)
        confidence_topic = depth_image_topic.replace('depth', 'confidence')
        self.sub_conf = rospy.Subscriber(confidence_topic, msg_Image, self.confidenceCallback)
        self.sub_point =  rospy.Subscriber(point_info_topic, four_point, self.four_pix_Callback)
        self.camera_to_QR = rospy.Subscriber(tf_transform,AprilTagDetectionArray,self.QR_Callback)
        self.listener = tf.TransformListener()
        self.intrinsics = None
        self.pix = None
        self.pix_grade = None
        self.pix_list=None
        self.image_info=None
        self.pix_QR=None
        self.result=None
        self.aver_list=None
    
    def transform_point(self,result_T,result_P,aver_list):  #坐标变换函数，未完成
        br = tf.TransformBroadcaster()
        br.sendTransform(result_T,result_P,rospy.Time.now(),"QR","camera")
        result = [0,0,0]
        for aver in aver_list:
            self.listener.transformPoint()
            
   
    def QR_Callback(self,data):  #得到二维码的坐标
        aver=self.avel_list
        self.pix_QR[0]=data.detections[0].ImagePoint[0]
        self.pix_QR[1]=data.detections[0].ImagePoint[1]     
        result_T=[0,0,0]
        result_P=[0,0,0,1]
        
        result_T[0]=data.detections[0].pose.pose.pose.position.x*1000
        result_T[1]=data.detections[0].pose.pose.pose.position.y*1000
        result_T[2]=data.detections[0].pose.pose.pose.position.z*1000   #得到二维码的平移坐标
        

        result_P[0]=data.detections[0].pose.opse.pose.orientation.x
        result_P[1]=data.detections[0].pose.opse.pose.orientation.y
        result_P[2]=data.detections[0].pose.opse.pose.orientation.z
        result_P[3]=data.detections[0].pose.opse.pose.orientation.w      #得到二维码的旋转坐标
        
    def AVER_P(self,image_lnfo,pix_list,x_center,y_center): #计算出一个有效的深度直
        aver_p = 10000
        point_reault = None
        for m in range(0,10):
	    try:
                if(len(pix_list)>5):
                    points_D = random.sample(range(0,10),5)
                    num_p=5
                else:
                    points_D = random.sample(range(0,10),4)
                    num_p=4
            except:
                print("Don't have enough points")
                return
            sum_p = float(0) 
            for i in range(0,len(points_D)):
                sum_p = sum_p + int(image_lnfo["image"][int(pix_list[points_D[i]][1]),int(pix_list[points_D[i]][0])])
            #aver_p = sum_p//5    
            if aver_p > sum_p//num_p:
                aver_p = sum_p//num_p
                point_result = points_D
            if sum_p < 30:      
                break   #在十次计算值中找到一个最佳的深度直
        
        aver = rs2.rs2_deproject_pixel_to_point(image_lnfo["image_intrinsics"], [x_center,y_center], aver_p)
        return aver    

    def SUM_E(self,point,image_lnfo):  #计算ROI区域含深度直点的深度平均值
        ROIX_S = int(point.data[1]+0.4*(point.data[3]-point.data[1]))
        ROIX_B = int(point.data[1]+0.8*(point.data[3]-point.data[1]))
        ROIY_S = int(point.data[0]+0.4*(point.data[2]-point.data[0]))
        ROIY_B = int(point.data[0]+0.6*(point.data[2]-point.data[0]))
            
        ROI = image_lnfo["image"][ROIY_S:ROIY_B,ROIX_S:ROIX_B]
        ROI_size = ROI.shape
        ROI_w = ROI_size[0]
        ROI_h = ROI_size[1]  #计算出ROI区域的属性
        
        cv2.imshow("ROI_Image",ROI)
        cv2.waitKey(1)
    
        t=0
        sum_E = 0
        for x in range(0,ROI_w):
            for y in range(0,ROI_h):
                if ROI[x,y]!=0:
                    sum_E = sum_E+ROI[x,y]
                    t=t+1
        sum_E = int(sum_E) // int(t)
        print("sum_E = "+str(sum_E))
        return(sum_E)   #计算深度平均值
    
    def four_pix_Callback(self,back_data):  
        pixes_list = []
        self.aver_list = []
        image_lnfo = self.image_lnfo
        num = 1
        for point in back_data.four_Data:
            print('car number---'+" "+str(num))
            pix_list = []
            i=0
            x_center=int(0.5*(point.data[1]+point.data[3]))
            y_center=int(0.5*(point.data[0]+point.data[2]))
            print("(x_center,y_center) = (%d,%d)"%(x_center,y_center))
	    while(len(pix_list) < 10 and i<1000):
		x=x_center+i
		y1=y_center+i
		y2=y_center-i
		for m in range(0,2*i):
		    x=x-m
		    if image_lnfo["image"][y1,x] != 0: 
		            pix_list.append([x,y1])
		    if image_lnfo["image"][y2,x] != 0:
		            pix_list.append([x,y2])
                i=i+1
            
            sum_E = self.SUM_E(point,image_lnfo)  
            aver=self.AVER_P(image_lnfo,pix_list,x_center,y_center)
            aver_list.append(aver)
            #print('-----------------------------'+str(ROI[1]))
            
            print('pix_list_len = ' +str(len(pix_list)))
            pixes_list.append(pix_list) 
            print('-------------------------------------------------------------------')
            num=num+1
        self.aver_list = aver_list
        print('pixes_len = ' +str(len(pixes_list)))  
        print('---------'+'\n')
        print('\n')
            
 
    def imageDepthCallback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            self.image_lnfo={"image_intrinsics":self.intrinsics,"image":cv_image}
            
        except CvBridgeError as e:
            print(e)
            return
        except ValueError as e:
            return

    def confidenceCallback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            grades = np.bitwise_and(cv_image >> 4, 0x0f)
            if (self.pix):
                self.pix_grade = grades[self.pix[1], self.pix[0]]
        except CvBridgeError as e:
            print(e)
            return



    def imageDepthInfoCallback(self, cameraInfo):
        try:
            if self.intrinsics:
                return
            self.intrinsics = rs2.intrinsics()
            self.intrinsics.width = cameraInfo.width
            self.intrinsics.height = cameraInfo.height
            self.intrinsics.ppx = cameraInfo.K[2]
            self.intrinsics.ppy = cameraInfo.K[5]
            self.intrinsics.fx = cameraInfo.K[0]
            self.intrinsics.fy = cameraInfo.K[4]
            if cameraInfo.distortion_model == 'plumb_bob':
                self.intrinsics.model = rs2.distortion.brown_conrady
            elif cameraInfo.distortion_model == 'equidistant':
                self.intrinsics.model = rs2.distortion.kannala_brandt4
            self.intrinsics.coeffs = [i for i in cameraInfo.D]
        except CvBridgeError as e:
            print(e)
            return

def main():
    depth_image_topic = '/camera/aligned_depth_to_color/image_raw'
    depth_info_topic = '/camera/aligned_depth_to_color/camera_info'
    point_info_topic = 'Ro_four_point'
    tf_transform = 'tag_detections'
    print ()
    print ('show_center_depth.py')
    print ('--------------------')
    print ('App to demontrate the usage of the /camera/aligned_depth_to_color/ topics.')
    print ()
    print ('Application subscribes to %s and %s topics.' % (depth_image_topic, depth_info_topic))
    print ('Application then calculates and print the range to the closest object.')
    print ('If intrinsics data is available, it also prints the 3D location of the object')
    print ('If a confedence map is also available in the topic %s, it also prints the confidence grade.' % depth_image_topic.replace('depth', 'confidence'))
    print ()
    
    listener = ImageListener(depth_image_topic, depth_info_topic,point_info_topic,tf_transform)
    rospy.spin()

if __name__ == '__main__':
    node_name = os.path.basename(sys.argv[0]).split('.')[0]
    rospy.init_node(node_name)
    main()

