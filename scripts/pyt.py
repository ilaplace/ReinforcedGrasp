#!/usr/bin/env python
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from tut.msg import dof

import rospy


def talker():
    arr = dof()
    pub = rospy.Publisher('burlap_state', dof, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    dof_list = [1,1,1]
    while not rospy.is_shutdown():
        arr.dofs = dof_list
        rospy.loginfo(arr.dofs)
        pub.publish(arr)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

