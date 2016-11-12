#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray, String, Int16

rospy.init_node('translate', anonymous=True)
pub = rospy.Publisher("arduino_commands", Int16, queue_size=10)


odwallas = [3,4,5,6]
sprite = [7,8,9,10,11,12]
water = [13,14,15]

#odwallas = [260,261,262,263,264,265,266,267,268,269]

def callback(objects):
    if (objects.data):
        obj_id = objects.data[0]
        rospy.loginfo(rospy.get_caller_id() + str(obj_id))
        print(obj_id)
        make_decisions(obj_id)



def make_decisions(obj_id):
    rate = rospy.Rate(10)

    rospy.loginfo(1)
    if (obj_id in odwallas):
        pub.publish(1)
    elif (obj_id in sprite):
        pub.publish(2)
    elif (obj_id in water):
        pub.publish(3)
    else:
        pub.publish(0)



def listener():
    rospy.Subscriber("objects", Float32MultiArray, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()



if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
