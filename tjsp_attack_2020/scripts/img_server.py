#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import rospy
import numpy as np
from tjsp_attack_2020.srv import img
from tensorflow.keras.models import load_model
import tensorflow as tf
from PIL import Image
from time import time

imag_num = 1


# 默认情况下服务使用的是多线程模式，TensorFlow在不同线程中加载和使用模型会出错，使用全局默认graph方法解决
global graph,model
graph = tf.get_default_graph()

#加载模型
model_path = os.path.dirname(os.path.dirname(__file__)) + '/model/classify.h5'
load_model = load_model(model_path)

def imgCallback(req):
    global imag_num, time_0
	# 将数组排列为图片
    # req.img为 Ascii 码，需要转换
    img = [ord(i) for i in req.img]
    img = np.array(np.uint8(img))
    img = img.reshape((req.height, req.width, req.channels))
    img = Image.fromarray(img[:,:,::-1])
    # img.show()
    img = img.resize((32, 32))
    img = np.array(img) / 255.
    img = np.expand_dims(img, axis = 0)

    if imag_num == 1:
        time_0 = time()
    print "处理图片数：%d，时间：%f" % (imag_num, time() - time_0)
    
    imag_num += 1
    #预测并反馈数据
    with graph.as_default():
        return np.argmax(load_model.predict(img), 1)[0]
    
def img_server():
	# ROS节点初始化
    rospy.init_node('img_server')

	# 创建一个名为/classify的server，注册回调函数imgCallback
    s = rospy.Service('/classify', img, imgCallback)

	# 循环等待回调函数
    rospy.spin()

if __name__ == "__main__":
    img_server()