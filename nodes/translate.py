#! /bin/env python3

import rospy
from turtle_control.msg import TurtleVelocity
from geometry_msgs.msg import Twist

class Translate:
    def __init__(self):
        self.sub = rospy.Subscriber("turtle_cmd", TurtleVelocity, self.callback);
        self.pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=1);
    def callback(self, vel):
        rospy.loginfo(vel);
        t = Twist();
        t.linear.x = vel.linear;
        t.angular.z = vel.angular;
        print("publish cmd_vel");
        self.pub.publish(t);
def main():
    rospy.init_node("translate");
    instance = Translate();
    rospy.spin();
if __name__ == '__main__':
    try:
        main();
    except rospy.ROSInterruptException:
        pass
