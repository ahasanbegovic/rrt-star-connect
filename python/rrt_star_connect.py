#!/usr/bin/env python

# TO DO: Separate classes and associated functions in different scripts

import math
import rospy
import random
import time
import os


from scipy import spatial
from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PointStamped, Point, Pose, PoseArray
from std_msgs.msg import Bool
from operator import attrgetter

def write_data_to_file(data_list, filename):
    f = open(filename, mode='w')
    f.write('iteration,nodes_origin,nodes_goal,path_length,exploration_duration\n')
    for line_list in data_list:
        line = ",".join([str(element) for element in line_list])
        f.write(line + '\n')
    f.close()


class Wrapper(object):
    def __init__(self, mapa):
        self.width_pixels = mapa.info.width
        self.length_pixels = mapa.info.height
        self.origin_x= mapa.info.origin.position.x
        self.origin_y= mapa.info.origin.position.y
        self.length_m = (abs (mapa.info.origin.position.x))*2
        self.width_m = (abs (mapa.info.origin.position.y))*2
        self.matrix_of_pixels = [[True if j!=0 else False for j in mapa.data[i*self.length_pixels:i*self.length_pixels+self.length_pixels]] for i in reversed(xrange(self.width_pixels))]

    def colision(self,x,y):
        pixel_x_left=int(self.width_pixels*(y-0.3-self.origin_y)/(-self.width_m))
        pixel_x_right=int(self.width_pixels*(y+0.3-self.origin_y)/(-self.width_m))
        pixel_y_down=int(self.length_pixels*(x-0.3-self.origin_x)/self.length_m)
        pixel_y_up=int(self.length_pixels*(x+0.3-self.origin_x)/self.length_m)
        pixel_x=int(self.width_pixels*(y-self.origin_y)/(-self.width_m))
        pixel_y=int(self.length_pixels*(x-self.origin_x)/self.length_m)
        if abs(pixel_x) > len(self.matrix_of_pixels[0])-1 or abs(pixel_y) > len(self.matrix_of_pixels)-1:
            return True
        elif abs(pixel_y_down) > len(self.matrix_of_pixels)-1 or abs(pixel_y_up) > len(self.matrix_of_pixels)-1:
            return True
        elif abs(pixel_x_right) > len(self.matrix_of_pixels[0])-1 or abs(pixel_x_left) > len(self.matrix_of_pixels[0])-1:
            return True
        else:
            first = self.matrix_of_pixels[pixel_x][pixel_y_down] or self.matrix_of_pixels[pixel_x][pixel_y_up]
            second=self.matrix_of_pixels[pixel_x_left][pixel_y] or self.matrix_of_pixels[pixel_x_right][pixel_y]
            third=self.matrix_of_pixels[pixel_x_left][pixel_y_up] or self.matrix_of_pixels[pixel_x_left][pixel_y_down]
            fourth=self.matrix_of_pixels[pixel_x_right][pixel_y_up] or self.matrix_of_pixels[pixel_x_right][pixel_y_down]
            return (self.matrix_of_pixels[pixel_x][pixel_y] or first or second or third or fourth)

    def colision_join(self,x,y):
        pixel_x=int(self.width_pixels*(y-self.origin_y)/(-self.width_m))
        pixel_y=int(self.length_pixels*(x-self.origin_x)/self.length_m)
        return self.matrix_of_pixels[pixel_x][pixel_y]

    def get_length_x(self):
        return self.length_m
    def get_width_y(self):
        return self.width_m

class Node (object):
    def __init__(self, data, parent=None):
        self.data=data
        self.parent=parent
        self.distance_traveled=0.0
        if parent != None:
            parent.children.append(self)
            self.distance_traveled = parent.distance_traveled + euclidian_distance(self.data, parent.data)
        self.children = []

    def __iter__(self):
        node=self
        while node != None:
            yield node
            node=node.parent

    def change_parent(self, new_parent, previous_parent=None):
    	if previous_parent != None:
    		previous_parent.children.remove(self)
    	self.parent = new_parent
        self.distance_traveled = self.parent.distance_traveled + euclidian_distance(self.data, self.parent.data)
    	new_parent.children.append(self)


class Tree(object):
    def __init__(self,initial_node):
        self.list_of_nodes=[]
        self.list_of_points=[]
        self.list_of_nodes.append(initial_node)
        self.list_of_points.append(initial_node.data)

    def dodaj (self,Node):
        self.list_of_nodes.append(Node)
        self.list_of_points.append(Node.data)


x_goal=None
y_goal=None
x_origin=0.0
y_origin=-0.5
wrapper=None
epsilon=0.2
max_distance_neighbors = 2.0
done=0
stop=None

output_file = os.path.expanduser("~/catkin_ws/src/rrt_star_connect/test_results/")

instance = 0

def read_map(pose):
    global wrapper
    wrapper=Wrapper(pose)

def init(pose):
    global x_origin
    global y_origin
    x_origin=pose.pose.pose.position.x
    y_origin=pose.pose.pose.position.y
    q=pose.pose.pose.orientation

def goal(pose):
    global x_goal
    global y_goal
    x_goal=pose.point.x
    y_goal=pose.point.y
    halt=Bool()
    halt.data=True
    stop.publish(halt)
    rrt()

def gotovo(nodes):
    final_path=Marker()
    final_path.type=4
    final_path.header.frame_id='odom'
    final_path.scale.x=0.3
    final_path.color.g=1.0
    final_path.color.a=1.0

    pose_array=PoseArray()
    for i in xrange(0,len(nodes)):
        path_point=Point()
        path_point_pose=Pose()
        path_point_pose.position.x=nodes[i][0]
        path_point_pose.position.y=nodes[i][1]
        path_point.x=nodes[i][0]
        path_point.y=nodes[i][1]
        final_path.points.append(path_point)
        pose_array.poses.append(path_point_pose)

    final_path_pub.publish(final_path)

    send_pose_array.publish(pose_array)
    rate.sleep()


def euclidian_distance(point1,point2):
    return math.sqrt((point2[1] - point1[1]) ** 2 + (point2[0] - point1[0]) ** 2)

def new_point_between(p, q, alpha):
    assert len(p) == len(q), 'Points must have the same dimension'
    return tuple([(1 - alpha) * p[i] + alpha * q[i] for i in xrange(len(p))])

def join_rrt(p1,p2):
    global epsilon
    global wrapper
    global max_distance_neighbors

    d=euclidian_distance(p1,p2)

    if d > max_distance_neighbors:
        p2 = new_point_between(p1,p2,max_distance_neighbors/d)
        d = max_distance_neighbors

    if d<epsilon:
        return None

    alpha_i=epsilon/d
    n=int(d/epsilon)

    memorize=None
    for i in xrange(1,n):
        alpha=alpha_i*i
        new_point=new_point_between(p1,p2,alpha)
        if wrapper.colision(float(new_point[0]),float(new_point[1])):
            return memorize
        else:
            memorize=new_point
    return memorize

def join_prm(p1,p2):
    global epsilon
    global wrapper

    d=euclidian_distance(p1,p2)

    if d<epsilon:
        return p2

    alpha_i=epsilon/d
    n=int(d/epsilon)

    memorize=None
    for i in xrange(1,n):
        alpha=alpha_i*i
        new_point=new_point_between(p1,p2,alpha)
        if wrapper.colision(float(new_point[0]),float(new_point[1])):
            return None
        else:
            memorize=new_point
    return memorize

def rrt():
    global x_goal
    global y_goal
    global wrapper
    global epsilon
    global max_distance_neighbors
    global final_path_pub
    global done
    global x_origin
    global y_origin

    global instance

    
    number_iteration=10000
    nodes_origin_pub = rospy.Publisher('/nodes_origin', Marker, queue_size=10)
    tree_origin_pub = rospy.Publisher('/branches_origin', Marker, queue_size=10)
    nodes_goal_pub = rospy.Publisher('/nodes_goal', Marker, queue_size=10)
    tree_goal_pub = rospy.Publisher('/branches_goal', Marker, queue_size=10)

    rate = rospy.Rate(10)
    
    marker_nodes_origin=Marker()
    marker_nodes_origin.type=8
    marker_nodes_origin.header.frame_id='odom'
    marker_nodes_origin.scale.x=0.2
    marker_nodes_origin.scale.y=0.2
    marker_nodes_origin.color.r = 1.0
    marker_nodes_origin.color.a = 1.0

    marker_nodes_goal=Marker()
    marker_nodes_goal.type=8
    marker_nodes_goal.header.frame_id='odom'
    marker_nodes_goal.scale.x=0.2
    marker_nodes_goal.scale.y=0.2
    marker_nodes_goal.color.b = 1.0
    marker_nodes_goal.color.a = 1.0

    marker_branches_origin=Marker()
    marker_branches_origin.type=5
    marker_branches_origin.header.frame_id='odom'
    marker_branches_origin.scale.x=0.2
    marker_branches_origin.color.b=1.0
    marker_branches_origin.color.a=1.0

    marker_branches_goal=Marker()
    marker_branches_goal.type=5
    marker_branches_goal.header.frame_id='odom'
    marker_branches_goal.scale.x=0.2
    marker_branches_goal.color.r=1.0
    marker_branches_goal.color.a=1.0


    # RRT Inicijaliziranje stabala
    origin_point = Node((x_origin, y_origin))
    goal_point = Node((float(x_goal), float(y_goal)))
    tree_origin = Tree((origin_point))
    tree_goal = Tree((goal_point))

    point_link_origin = tree_origin.list_of_points[0]
    origin_exploitation = point_link_origin
    index_origin = 0
    point_link_goal = tree_goal.list_of_points[0]
    goal_exploitation = point_link_goal
    index_goal = 0 

    path_length = wrapper.length_m * wrapper.width_m

    output_data = []

    for i in range(number_iteration):
        nodes_origin=Point()
        nodes_goal = Point()
        exploration_duration = 0

        exploration = random.uniform(0.0, 1.0)
        if exploration < 0.8:
            start_time = time.time()

            rand=(random.uniform(float(wrapper.origin_x),float(wrapper.length_m/2)),random.uniform(float(wrapper.origin_y),float(wrapper.width_m/2)))
    	    distance, index = spatial.KDTree(tree_origin.list_of_points).query(rand)
            point_link_origin = tree_origin.list_of_points[index]
            new_point_origin = join_rrt(point_link_origin, rand)

            if new_point_origin != None:

                indices_of_neighboring_points_origin = spatial.KDTree(tree_origin.list_of_points).query_ball_point(new_point_origin, max_distance_neighbors)
                new_point_origin_distance_traveled_min = 1600.0
                index_min = index
                for index_neigboring_points_origin in indices_of_neighboring_points_origin:
                    neighboring_node_origin = tree_origin.list_of_nodes[index_neigboring_points_origin]
                    distance_neighbors_origin = euclidian_distance(new_point_origin, neighboring_node_origin.data)
                    if join_prm(new_point_origin, neighboring_node_origin.data) is not None and neighboring_node_origin.distance_traveled + distance_neighbors_origin < new_point_origin_distance_traveled_min:
                        index_min = index_neigboring_points_origin
                        new_point_origin_distance_traveled_min = neighboring_node_origin.distance_traveled + distance_neighbors_origin

                new_node_origin = Node(new_point_origin, tree_origin.list_of_nodes[index_min])
                tree_origin.dodaj(new_node_origin)
                nodes_origin.x = new_point_origin[0]
                nodes_origin.y = new_point_origin[1]
                marker_nodes_origin.points.append(nodes_origin)
                nodes_origin_pub.publish(marker_nodes_origin)


                for index_neigboring_points_origin in indices_of_neighboring_points_origin:
                    neighboring_node_origin = tree_origin.list_of_nodes[index_neigboring_points_origin]
                    distance_neighbors_origin = euclidian_distance(new_node_origin.data, neighboring_node_origin.data)
                    if neighboring_node_origin.distance_traveled > new_node_origin.distance_traveled + distance_neighbors_origin and distance_neighbors_origin <= max_distance_neighbors:
                        linking_point_origin = join_rrt(new_node_origin.data, neighboring_node_origin.data)
                        if linking_point_origin is not None and euclidian_distance(linking_point_origin, neighboring_node_origin.data) <= 2 * epsilon:
                            previous_parent_index = tree_origin.list_of_nodes.index(neighboring_node_origin.parent)
                            neighboring_node_origin.change_parent(new_node_origin, previous_parent=tree_origin.list_of_nodes[previous_parent_index])


                distance_origin_goal, index_origin_goal = spatial.KDTree(tree_goal.list_of_points).query(new_point_origin)
                if (distance_origin_goal + new_node_origin.distance_traveled + tree_goal.list_of_nodes[index_origin_goal].distance_traveled < path_length):
                    origin_exploitation = new_point_origin
                    index_origin = len(tree_origin.list_of_points)-1
                    goal_exploitation = tree_goal.list_of_points[index_origin_goal]
                    index_goal = index_origin_goal

            marker_branches_origin.points = []
            for node_a in tree_origin.list_of_nodes:
                a = Point()
                a.x = node_a.data[0]
                a.y = node_a.data[1]
                for node_b in node_a.children:
                    b = Point()
                    b.x = node_b.data[0]
                    b.y = node_b.data[1]

                    marker_branches_origin.points.append(a)
                    marker_branches_origin.points.append(b)
            tree_origin_pub.publish(marker_branches_origin)


            rand1=(random.uniform(float(wrapper.origin_x),float(wrapper.length_m/2)),random.uniform(float(wrapper.origin_y),float(wrapper.width_m/2)))
            distance1, index1 = spatial.KDTree(tree_goal.list_of_points).query(rand1)
            point_link_goal = tree_goal.list_of_points[index1]
            new_point_goal = join_rrt(point_link_goal, rand1)

            if new_point_goal != None:

                indices_of_neighboring_points_goal = spatial.KDTree(tree_goal.list_of_points).query_ball_point(new_point_goal, max_distance_neighbors)

                new_point_goal_distance_traveled_min = 1600.0
                index1_min = index1
                for index_neigboring_points_goal in indices_of_neighboring_points_goal:
                    neighboring_node_goal = tree_goal.list_of_nodes[index_neigboring_points_goal]
                    distance_neighbors_goal = euclidian_distance(new_point_goal, neighboring_node_goal.data)
                    if join_prm(new_point_goal, neighboring_node_goal.data) is not None and neighboring_node_goal.distance_traveled + distance_neighbors_goal < new_point_goal_distance_traveled_min:
                        index1_min = index_neigboring_points_goal
                        new_point_goal_distance_traveled_min = neighboring_node_goal.distance_traveled + distance_neighbors_goal



                new_node_goal = Node(new_point_goal, tree_goal.list_of_nodes[index1_min])
                tree_goal.dodaj(new_node_goal)
                nodes_goal.x = new_point_goal[0]
                nodes_goal.y = new_point_goal[1]
                marker_nodes_goal.points.append(nodes_goal)
                nodes_goal_pub.publish(marker_nodes_goal)


                for index_neigboring_points_goal in indices_of_neighboring_points_goal:
                    neighboring_node_goal = tree_goal.list_of_nodes[index_neigboring_points_goal]
                    distance_neighbors_goal = euclidian_distance(new_node_goal.data, neighboring_node_goal.data)
                    if neighboring_node_goal.distance_traveled > new_node_goal.distance_traveled + distance_neighbors_goal and distance_neighbors_goal <= max_distance_neighbors:
                        linking_point_goal = join_rrt(new_node_goal.data, neighboring_node_goal.data)
                        if linking_point_goal is not None and euclidian_distance(linking_point_goal, neighboring_node_goal.data) <= 2 * epsilon:
                            previous_parent_index = tree_goal.list_of_nodes.index(neighboring_node_goal.parent)
                            neighboring_node_goal.change_parent(new_node_goal, previous_parent=tree_goal.list_of_nodes[previous_parent_index])


                distance_goal_origin, index_goal_origin = spatial.KDTree(tree_origin.list_of_points).query(new_point_goal)
                if (distance_goal_origin + new_node_goal.distance_traveled + tree_origin.list_of_nodes[index_goal_origin].distance_traveled < path_length):
                    goal_exploitation = new_point_goal
                    index_goal = len(tree_goal.list_of_points)-1
                    origin_exploitation = tree_origin.list_of_points[index_goal_origin]
                    index_origin = index_goal_origin

            marker_branches_goal.points = []
            for node_a in tree_goal.list_of_nodes:
                a = Point()
                a.x = node_a.data[0]
                a.y = node_a.data[1]
                for node_b in node_a.children:
                    b = Point()
                    b.x = node_b.data[0]
                    b.y = node_b.data[1]

                    marker_branches_goal.points.append(a)
                    marker_branches_goal.points.append(b)
            tree_goal_pub.publish(marker_branches_goal)

            exploration_duration = time.time() - start_time
        else:
            new_point_pk = join_prm(origin_exploitation, goal_exploitation)
            if new_point_pk != None:
                send_origin = [node.data for node in tree_origin.list_of_nodes[index_origin]]
                send_origin = send_origin[::-1]
                send_goal = [node.data for node in tree_goal.list_of_nodes[index_goal]]
                send = []
                send.extend(send_origin)
                send.extend(send_goal)
                gotovo(send)

                path_length = euclidian_distance(origin_exploitation, goal_exploitation) + \
                 tree_origin.list_of_nodes[tree_origin.list_of_points.index(origin_exploitation)].distance_traveled + \
                  tree_goal.list_of_nodes[tree_goal.list_of_points.index(goal_exploitation)].distance_traveled
                #print "---"
                #print "END"
                #break
        print "-----"
        print "I: ", i
        print "CP:", len(tree_origin.list_of_nodes)
        print "CK:", len(tree_goal.list_of_nodes)
        print "D: ", path_length
        print "t: ", exploration_duration

        output_data.append([i, len(tree_origin.list_of_nodes), len(tree_goal.list_of_nodes), path_length, exploration_duration])

        rate.sleep()
    write_data_to_file(output_data, output_file+"rrt_star_connect_"+str(instance)+".txt")
    instance += 1


            



if __name__ == '__main__':
    rospy.init_node('rrt_star_connect', anonymous=True)
    rate=rospy.Rate(10)
    rospy.Subscriber("/goal_configuration", PointStamped, goal)
    rospy.Subscriber("/map", OccupancyGrid, read_map)
    rospy.Subscriber("/base_pose_ground_truth", Odometry, init)
    final_path_pub=rospy.Publisher('/path', Marker, queue_size=10)
    send_pose_array=rospy.Publisher('/goals',PoseArray,queue_size=10)
    stop=rospy.Publisher('/halting_robot',Bool,queue_size=10)

    while wrapper is None or x_goal is None:
        rate.sleep()

    while not rospy.is_shutdown():
        rospy.spin()
