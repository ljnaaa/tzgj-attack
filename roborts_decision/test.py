import rospy



if __name__ == "__main__":
    rospy.init_node("test")
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        print("123")
        rate.sleep()