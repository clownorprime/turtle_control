#! /bin/env python3

import rospy
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute, TeleportAbsoluteRequest, SetPen
from turtle_control.srv import Start
from turtle_control.msg import TurtleVelocity
from std_srvs.srv import Empty
from math import sqrt, atan2
from time import sleep

class Follwer:
    def __init__(self):
        self.restart_service = rospy.Service("restart", Start, self.restart_handle);
        self.pose_sub = rospy.Subscriber("/turtle1/pose", Pose, self.pose_callback);
        self.turtleVelocity_pub = rospy.Publisher("turtle_cmd", TurtleVelocity, queue_size = 1);
        self.draw = rospy.ServiceProxy("/turtle1/draw", Empty);
        self.jump = rospy.ServiceProxy("/turtle1/teleport_absolute", TeleportAbsolute);
        self.waypoints = rospy.get_param("/waypoints");
        self.index = 0;
        self.start = False;   #the turtle shouldn't start moving until the restart reserivce has been called; 
        self.set_pen = rospy.ServiceProxy("/turtle1/set_pen", SetPen);
        self.turn_pen_off();
    def turn_pen_off(self):
        self.set_pen(255, 255, 0, 2, 1);
    def turn_pen_on(self):
        self.set_pen(255, 255, 0, 2, 0);
    def pose_callback(self, pose):
        if (not self.start):
            return;
        self.turn_pen_on();
        dist_thresh = rospy.get_param("dist_thresh");
        target_point = self.waypoints[self.index];
        start_point = [pose.x, pose.y];
        start_theta = pose.theta;
        print("target_point: [" + str(target_point[0]) + " " + str(target_point[1]) + str("]"));
        print("start_point: [" + str(start_point[0]) + " " + str(start_point[1]) + str("]"));
        target_distance = sqrt((target_point[0] - start_point[0]) ** 2 +(target_point[1] - start_point[1]) ** 2);
        if (target_distance > dist_thresh):
            target_theta = atan2((target_point[1] - start_point[1]), (target_point[0] - start_point[0]));
            if (abs(target_theta - start_theta) > 0.1):
                vel = TurtleVelocity();
                vel.linear = 0;
                vel.angular = 3;
                self.turtleVelocity_pub.publish(vel);
            else:
                vel = TurtleVelocity();
                vel.linear = 2;
                vel.angular = 0;
                self.turtleVelocity_pub.publish(vel);
        else:
            print("change to another point");
            self.index = (self.index + 1) % (len(self.waypoints));
        return;
    def restart_handle(self, start):
        # note the order of call and self.start is really important.
        # we need to first setup the map, then set start to be true.
        # so we can enable the order of the setup and pose_callback.
        self.draw.call();
        self.start = True;
        x, y = start.x, start.y;
        # reset the position of turtle to it's starting position.
        self.jump.call(TeleportAbsoluteRequest(x, y, 0));
        distance = 0;
        for point in self.waypoints:
            distance += sqrt(pow(x - point[0], 2) + pow(y - point[1], 2));
            x, y = point[0], point[1];
        return distance;

def main():
    rospy.init_node("follower");
    instance = Follwer();
    rospy.spin();

if __name__ == '__main__':
    try:
        main();
    except rospy.ROSInterruptException:
        pass
