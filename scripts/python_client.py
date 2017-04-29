#!/usr/bin/env python
import rospy
from graspit_commander import GraspitCommander
from std_msgs.msg import String
from tut.msg import dof
import threading

actions = ["ACTION_P0_PLUS", "ACTION_P1_PLUS", "ACTION_P2_PLUS", "ACTION_P3_PLUS",
           "ACTION_P4_PLUS", "ACTION_P5_PLUS", "ACTION_P6_PLUS", "ACTION_P7_PLUS",
           "ACTION_P8_PLUS", "ACTION_P9_PLUS", "ACTION_P10_PLUS", "ACTION_P11_PLUS",
           "ACTION_P12_PLUS","ACTION_P13_PLUS", "ACTION_P14_PLUS", "ACTION_P15_PLUS",
           "ACTION_P0_MIN", "ACTION_P1_MIN", "ACTION_P2_MIN", "ACTION_P3_MIN",
           "ACTION_P4_MIN", "ACTION_P5_MIN", "ACTION_P6_MIN", "ACTION_P7_MIN",
           "ACTION_P7_MIN", "ACTION_P8_MIN", "ACTION_P9_MIN", "ACTION_P10_MIN",
           "ACTION_P11_MIN", "ACTION_P12_MIN", "ACTION_P13_MIN", "ACTION_P14_MIN",
           "ACTION_P15_MIN", "action1", "action2"]
ACTION_STEP = 0.02


class Ebe:
    def __init__(self):
        self.action = None
        self.dofs = None
        self.key = None
        rospy.init_node('listener', anonymous=True)

    def callback(self, data):
        for i in range(len(actions)):
            if actions[i] == data.data:
                self.action = actions[i]
                self.key = i

    def action_listener(self):

        rospy.Subscriber("burlap_action", String, self.callback)
        rospy.sleep(1)
        if self.action is not None:
            print self.action
        rospy.spin()

    def act(self):
        robot = GraspitCommander.getRobot(0)
        self.dofs = robot.robot.dofs

        if self.key < 15:
            for i in range(15):
                if actions[i] == self.action:
                    self.dofs[i] += ACTION_STEP
        else:
            for i in range(15, len(actions)):
                if actions[i] == self.action:
                    self.dofs[i] -= ACTION_STEP

        robot.moveDOFToContacts(self.dofs, 2, True)


class QualityPublisher(threading.Thread):
    def run(self):
        grasp_quality = GraspitCommander.computeQuality()
        pub = rospy.Publisher('graspQuality', String, queue_size=3)
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            msg_str = str(grasp_quality.epsilon) + " " + str(grasp_quality.volume)
            pub.publish(msg_str)
            rate.sleep()


class RobotStatePublisher(threading.Thread):
    def run(self):
        arr = dof()
        pub = rospy.Publisher('burlap_state', dof, queue_size=10)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            #robot = GraspitCommander.getRobot(0)
            #dofs = list(robot.robot.dofs)
            arr.dofs = [2,1]
            pub.publish(arr)
            rate.sleep()

        
if __name__ == '__main__':
    r = Ebe()
    #QualityPublisher().start()
    RobotStatePublisher().start()
    #r.action_listener()
