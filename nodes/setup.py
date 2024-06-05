#! /bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute, SetPen
from std_srvs.srv import Empty, EmptyResponse
from time import sleep

class DrawService:
    def __init__(self):
        self.set_pen = rospy.ServiceProxy("/turtle1/set_pen", SetPen);
        self.jump = rospy.ServiceProxy("/turtle1/teleport_absolute", TeleportAbsolute);
        self.waypoints = rospy.get_param("waypoints");
        self.draw_service = rospy.Service("/turtle1/draw", Empty, self.draw_points);
        self.reset = rospy.ServiceProxy("/reset", Empty);
    def turn_pen_on(self):
        self.set_pen(255, 255, 255, 2, 0);
    def turn_pen_off(self):
        self.set_pen(255, 255, 255, 2, 1);
    def draw_points(self, empty):
        for point in self.waypoints:
            x, y = point[0], point[1];
            self.turn_pen_off();
            self.jump(x, y, 0);
            self.draw_x_shape(x, y, 0.3);
        return EmptyResponse();

    def draw_x_shape(self, x, y, size):
        self.turn_pen_on();
        self.jump(x - size, y - size, 0);
        self.jump(x + size, y + size, 0);
        self.jump(x, y, 0);
        self.jump(x + size, y - size, 0);
        self.jump(x - size, y + size, 0);
        self.turn_pen_off();

def main():
    rospy.init_node("draw_service");
    instance = DrawService();
    rospy.spin();

if __name__ == '__main__':
    try:
        main();
    except rospy.ROSInterruptException:
        pass

