#!/usr/bin/env python2
 # -*- coding: UTF-8 -*- 
from robot_detect.yolo import YOLO
import rospy
from cv_bridge import CvBridge
import numpy as np
import cv2
import PIL
from sensor_msgs.msg import Image
from realsense2_camera.msg import four_point
from realsense2_camera.msg import int_pub
yolo = YOLO()


class ImageListener:
    def __init__(self):
        self.point_list = None
        self.sub = rospy.Subscriber(
            "/camera/color/image_raw", Image, self.imageCB)

    def imageCB(self, msg):
        self.point_list = []
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        cv_image = cv_image[:, :, ::-1]
        PIL_image = PIL.Image.fromarray(cv_image)
        boxes, labels = yolo.detect_simple(PIL_image)
        print(boxes)  # [[top, left, bottom, right]]
        print(labels)
        if(len(boxes) > 0):
            box = boxes[0]
            for i in range(len(box)):
                box[i] = int(box[i])
            cv_image = cv2.UMat(cv_image).get()
            cv2.rectangle(cv_image, (int(box[1]), int(
                box[0])), (int(box[3]), int(box[2])), (0, 255, 0), 2)
            # cv2.line(cv_image,(5,5),(100,100),(0,255,0),2)
        cv2.imshow("test", cv_image)
        cv2.waitKey(1)
        self.point_list = boxes
        self.points_pub()
    
    def points_pub(self):
        pub = rospy.Publisher('Ro_four_point', four_point, queue_size=30)
        point_list = self.point_list
        boxes = four_point()
        for point in point_list:
            box = int_pub()
            for i in point:
                box.data.append(int(i))
            boxes.four_Data.append(box)
        pub.publish(boxes)   #将得到的多个长方形框的左上角和右下角顶点发布
def main():
    listener = ImageListener()
    print("start")
    rospy.spin()

if __name__ == "__main__":
    node = rospy.init_node("yolo_node")
    main()
