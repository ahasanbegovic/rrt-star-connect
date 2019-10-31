#!/usr/bin/env python

# Note: This is third-party sourced code. I did not write this part.

import math
import rospy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped

class Wrapper(object):
    def __init__(self, space_map):
        self.width_pixels = space_map.info.width
        self.length_pixels = space_map.info.height
        self.start_x= space_map.info.origin.position.x
        self.start_y= space_map.info.origin.position.y
        self.length_m = (abs (space_map.info.origin.position.x))*2
        self.width_m = (abs (space_map.info.origin.position.y))*2
        self.matrix_of_pixels = [[True if j!=0 else False for j in space_map.data[i*self.length_pixels:i*self.length_pixels+self.length_pixels]] for i in reversed(xrange(self.width_pixels))]



    def collision(self,x,y,z):
        pixel_x=int(self.width_pixels*(y-self.start_y)/(-self.width_m))
        pixel_y=int(self.length_pixels*(x-self.start_x)/self.length_m)
        return self.matrix_of_pixels[pixel_x][pixel_y]


x_goal=0.0
y_goal=0.0
memorize_x=0.0
memorize_y=0.0
wrapper=None
pub=None

def read_map(pose):
    global wrapper
    wrapper=Wrapper(pose)


def publish_goal_configuration(pose):
    global x_goal
    global y_goal
    global memorize_x
    global memorize_y
    global wrapper
    x_goal=pose.point.x
    y_goal=pose.point.y
    pointstamped = PointStamped()
    if wrapper.collision(float(x_goal),float(y_goal),0)==False:
        pointstamped.point.x=x_goal
        memorize_x=x_goal
        pointstamped.point.y=y_goal
        memorize_y=y_goal
        pub.publish(pointstamped)
    else:
        pointstamped.point.x=memorize_x
        pointstamped.point.y=memorize_y


    rate.sleep()

if __name__ == '__main__':
    rospy.Subscriber("/clicked_point", PointStamped, publish_goal_configuration)
    rospy.Subscriber("/map", OccupancyGrid, read_map)
    pub = rospy.Publisher('/goal_configuration', PointStamped, queue_size=200)
    rospy.init_node('goal_configuration_node', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.spin()
