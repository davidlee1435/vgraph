#!/usr/bin/env python

import rospy
import time
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Twist, Quaternion, Pose, Point
from std_msgs.msg import Header, ColorRGBA
import numpy as np
from scipy.spatial import ConvexHull
import pickle
import math
import tf
from sensor_msgs.msg import LaserScan
from math import pi, pow, atan2, sqrt, radians, copysign, degrees
from transform_utils import quat_to_angle, normalize_angle 

class Bot:
	# Starts with constants and sets up publishers and subscribers
	def __init__(self):
		self.botsize = 40/2
		rospy.init_node('bot', anonymous=True)
		rospy.on_shutdown(self.shutdown) 
		self.marker_pub = rospy.Publisher('vgraph_markers', Marker, queue_size=1)
		self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)


		self.tf_listener = tf.TransformListener()
		rospy.sleep(2) #give tf some time to fill buffer 
		self.odom_frame = '/odom' 
		try:
	    		self.tf_listener.waitForTransform(self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
	    		self.base_frame = '/base_footprint'
		except (tf.Exception, tf.ConnectivityException, tf.LookupException):
	    		try:
				self.tf_listener.waitForTransform(self.odom_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
				self.base_frame = '/base_link'
	    		except (tf.Exception, tf.ConnectivityException, tf.LookupException):
				rospy.loginfo("Cannot find transform between odom and base link or base footprint")
				rospy.signal_shutdown("tf Exception") 
		self.start = Point()
		(self.start, self.start_rot) = self.get_odom()
		self.start.x = 0 #delete later 
		self.start.y = 0 #delete later 

		self.pos = Point()
		self.update_odom() 


		rospy.sleep(0.5)
		self.rate = 2
		self.linear_speed = 0.2
		self.angular_speed = 0.2
		self.start_point = Point(0.0, 0.0, 0.0)
		self.goal_point = self._read_goal_point()
		self.r = rospy.Rate(self.rate)
		self.vertex_points = []
		self.traversable_edges = []
		self.expand_obstacles()
		self.connect_lines()
		
		path = self.get_shortest_path()
		self._draw_path(path)
		self.follow_path(path) 	

	def follow_path(self, path):
		prev = path[0]
		for curr in path[1:]:
			#face that point
			curr_point = Point(*curr) 
			self.face_pos(curr_point)
			self.translate_amt(self.euclidean_distance(Point(*prev), curr_point))
			self.update_odom() 
			#move the euclidean distance between curr pos and that point
			prev = curr
	
	def get_odom(self):
		try:
	    		(trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
		except (tf.Exception, tf.ConnectivityException, tf.LookupException):
	    		rospy.loginfo("TF Exception")
	    		return 
		return (Point(*trans), quat_to_angle(Quaternion(*rot)))


	def face_pos(self, target_pos):
		self.update_odom()
		print("Current angle: " + str(degrees(self.rot)))
		correct_angle = atan2(target_pos.y - self.pos.y, target_pos.x - self.pos.x)
		print("Correct angle to face: " + str(degrees(correct_angle)) + ", Rotating by: " + str(degrees(correct_angle - self.rot)))
		self.rotate_amt(correct_angle - self.rot)
		self.r.sleep()
		self.update_odom()
		print("Final rot: " + str(degrees(self.rot)))

	def rotate_amt(self, angle):
		move_cmd = Twist()
		angular_duration = math.fabs(angle / self.angular_speed)

		if angle < 0:
			move_cmd.angular.z = self.angular_speed * -1.0
		else:
			move_cmd.angular.z = self.angular_speed

		ticks = int(angular_duration * self.rate)
		for t in range(ticks):
			self.cmd_vel.publish(move_cmd)
			self.r.sleep()
		self.cmd_vel.publish(Twist())
	
	def euclidean_distance(self, goal, pos):
		return sqrt(pow((goal.x - pos.x), 2) + pow((goal.y - pos.y), 2)) 
	def update_odom(self): 
		(self.pos, self.rot) = self.get_odom()

	def translate_amt(self, goal_distance):
        	linear_duration = goal_distance / self.linear_speed
        	linear_duration = math.fabs(linear_duration)

        	move_cmd = Twist()
		if goal_distance < 0:
			move_cmd.linear.x = self.linear_speed * -1.0
		else:
			move_cmd.linear.x = self.linear_speed
        
		ticks = int(linear_duration * self.rate)

        	for t in range(ticks):
            		self.cmd_vel.publish(move_cmd)
            		self.r.sleep()
	        self.cmd_vel.publish(Twist())

	def get_shortest_path(self):
		adj_matrix = {}
		for p1, p2 in self.traversable_edges:
			p1_tup, p2_tup = (p1.x, p1.y, p1.z), (p2.x, p2.y, p2.z)
			if p1_tup not in adj_matrix:
				adj_matrix[p1_tup] = []
			if p2_tup not in adj_matrix:
				adj_matrix[p2_tup] = []

			adj_matrix[p1_tup].append(p2_tup)
			adj_matrix[p2_tup].append(p1_tup)
		
		start = (self.start_point.x, self.start_point.y, self.start_point.z)
		end = (self.goal_point.x, self.goal_point.y, self.goal_point.z)
		path = self.dijkstra(adj_matrix, start, end)
		return path
	
	def _draw_path(self, path):
		if len(path) < 2:
			raise Exception("Can't draw line with fewer than 2 points")
		prev = path[0]
		id_counter = 1000
		for curr in path[1:]:
			prev_point, curr_point = Point(*prev), Point(*curr)
			self.draw([prev_point, curr_point], id_counter, is_path=True)
			id_counter += 1		
			prev = curr
		
	def dijkstra(self, graph, start, end):
		dp = {start: (0, None)}
		curr = start
		seen = set()
	
		while curr != end:
			seen.add(curr)
			neighbors = graph[curr]
			
			for neighbor in neighbors:
				dist_to_neighbor = self._distance(curr, neighbor) + dp[curr][0]
				if neighbor not in dp or dp[neighbor][0] > dist_to_neighbor:
					dp[neighbor] = (dist_to_neighbor, curr) 
			curr = None
			min_dist = float('inf')
			for node in dp:
				if node not in seen and dp[node][0] < min_dist:
					min_dist = dp[node][0]
					curr = node
			if not curr:
				raise Exception("Can't find path")
		
		path = []
		while curr:
			path.append(curr)
			curr = dp[curr][1]
		return path[::-1]
		
	# Connects vertices to each other except when colliding with obstacle
	def connect_lines(self):
		flat_vpts = [self.start_point, self.goal_point] + [vertex for shape in self.vertex_points for vertex in shape]
		id_counter = 100
		for i, v1 in enumerate(flat_vpts):
			for j, v2 in enumerate(flat_vpts[i+1:]):
				if rospy.is_shutdown():
					return
				# skip pairs of vertices on the same shape
				should_continue = False
				for shape in self.vertex_points:
					if v1 in shape and v2 in shape:
						should_continue = True
				if should_continue:
					continue

				if not self._intersects_any_object(v1, v2, self.vertex_points):
					self.draw([v1, v2], id_counter)
					self.traversable_edges.append((v1, v2))
					id_counter += 1

	# Expands obstables from given map 
	def expand_obstacles(self):
		inputfile = "../data/world_obstacles.txt"
		fin = open(inputfile, "r")
		lines = fin.readlines()
		n = len(lines)
		if n < 1:
			return False
		num_obstacles = lines[0] 
		index = 1
		while index < n and not rospy.is_shutdown():
			if len(lines[index].split(" ")) == 1:
				num_vertices = int(lines[index])
				points = []
				for j in range(index + 1, index + num_vertices + 1):
					point = lines[j].split(" ")
					expanded = self.expand(int(point[0]), int(point[1]))
					points += expanded
				expanded_vertices = ConvexHull(points)

				vispts = []
				for k in expanded_vertices.vertices:
					pt = Point()
					pt.x = points[k][0]/100.0 
					pt.y = points[k][1]/100.0
					vispts.append(pt)
				vispts.append(vispts[0])
				self.draw(vispts, index)
				self.vertex_points.append(vispts[:-1])
				index += num_vertices + 1
		fin.close()

	# draws the markers given points 
	def draw(self, points, index, is_path=False):
		while not rospy.is_shutdown():
			marker = Marker()
    			marker.header.frame_id = "/map"
			marker.id = index
    			marker.type = marker.LINE_STRIP
			marker.lifetime = rospy.Duration(0)
    			marker.action = marker.ADD
   			marker.scale.x = 0.01 if not is_path else 0.1
    			marker.scale.y = 0
    			marker.scale.z = 0
    			marker.color.a = 1.0
    			marker.color.r = 1.0 if not is_path else 0.0
    			marker.color.g = 1.0
    			marker.color.b = 0.0
   			marker.pose.orientation.x = 0.0
    			marker.pose.orientation.y = 0.0
    			marker.pose.orientation.z = 0.0
    			marker.pose.orientation.w = 0.0
    			marker.pose.position.x = 0.0
    			marker.pose.position.y = 0.0
    			marker.pose.position.z = 0.0
    			marker.points = points #.append(second_line_point)
    			self.marker_pub.publish(marker)
    			rospy.sleep(0.5)
			break #return marker 

        # Expands the point to possible locations 
	def expand(self, x, y):
		botsize = self.botsize
		result = []
		result.append([x + botsize, y - botsize])
		result.append([x - botsize, y + botsize])
		result.append([x + botsize, y + botsize])
		result.append([x - botsize, y - botsize])
		return result  

	def _read_goal_point(self):
		with open('../data/goal.txt', 'r') as f:
			line = f.read()
			x, y = map(float, line.split())
			return Point(x/100.0, y/100.0, 0.0)
	
	def _intersects_any_object(self, v1, v2, vertex_points):
		for object_vertices in vertex_points:
			if self._intersects_object(v1, v2, object_vertices):
				return True
		return False
	
	def _intersects_object(self, v1, v2, object_vertices):
		if len(object_vertices) < 2:
			return False
		object_vertices = list(object_vertices) + [object_vertices[0]]
		prev = object_vertices[0]
		for curr in object_vertices[1:]:
			# skip if vertex is equal to vertex on shape
			if v1 == prev or v2 == prev or v1 == curr or v2 == curr:
				prev = curr
				continue
			if self._intersects_line(v1, v2, prev, curr):
				return True
			prev = curr
		return False
	

	def _intersects_line(self, v1, v2, v3, v4):
		# Return True if line segment v1-v2 intersects v3-v4
		o1 = self._orientation(v1, v2, v3)
		o2 = self._orientation(v1, v2, v4)
		o3 = self._orientation(v3, v4, v1)
		o4 = self._orientation(v3, v4, v2)

		return (o1 != o2 and o3 != o4) or \
		       (o1 == 0 and self._on_segment(v1, v2, v3)) or \
		       (o2 == 0 and self._on_segment(v1, v2, v4)) or \
		       (o3 == 0 and self._on_segment(v3, v4, v1)) or \
		       (o4 == 0 and self._on_segment(v3, v4, v2))

	def _on_segment(self, v1, v2, point):
		 return (point.x <= max(v1.x, v2.x) and point.x >= min(v1.x, v2.x) and point.y <= max(v1.y, v2.y) and point.y >= min(v1.y, v2.y))

	def _orientation(self, v1, v2, v3):
		# Returns if the direction of lines connecting v1-v2, v2-v3, and v3-v1 are collinear, clockwise, or counter-clockwise
		val = ((v2.y - v1.y) * (v3.x - v2.x) - (v2.x - v1.x) * (v3.y - v2.y))
		if val == 0:
			return 0 # colinear
		return 1 if val > 0 else -1
 
	def _distance(self, v1, v2):
		return math.sqrt((v1[0]-v2[0])**2 + (v1[1]-v2[1])**2 + (v1[2] -v2[2])**2)
	# Standard shutdown method 
	def shutdown(self):
	       	rospy.loginfo("Stopping the robot...")
        	#self.cmd_vel.publish(Twist())
	        rospy.sleep(1)

if __name__ == '__main__':
	try:
		Bot()
	except Exception as e:
		print(e)
		rospy.loginfo("Bot terminated")
