import rospy
import time
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion, Pose, Point
from std_msgs.msg import Header, ColorRGBA
import numpy as np
from scipy.spatial import ConvexHull

class Bot:
	# Starts with constants and sets up publishers and subscribers
	def __init__(self):
		self.botsize = 40/2
		rospy.init_node('bot', anonymous=True)
		rospy.on_shutdown(self.shutdown) 
		self.marker_pub = rospy.Publisher('vgraph_markers', Marker, queue_size=1)

		rospy.sleep(0.5)
		self.rate = 2
		self.start_point = Point(0.0, 0.0, 0.0)
		self.goal_point = self._read_goal_point()
		self.r = rospy.Rate(self.rate)
		self.vertex_points = []
		self.traversable_edges = []
		self.expand_obstacles()
		self.connect_lines()
	

	def get_shortest_path(self):
		adj_matrix = 	
	# Connects vertices to each other except when colliding with obstacle
	def connect_lines(self):
		flat_vpts = [self.start_point, self.goal_point] + [vertex for shape in self.vertex_points for vertex in shape]
		id_counter = 100
		for i, v1 in enumerate(flat_vpts):
			for j, v2 in enumerate(flat_vpts[i+1:]):
				if rospy.is_shutdown():
					return

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
	def draw(self, points, index):
		while not rospy.is_shutdown():
			marker = Marker()
    			marker.header.frame_id = "/map"
			marker.id = index
    			marker.type = marker.LINE_STRIP
			marker.lifetime = rospy.Duration(0)
    			marker.action = marker.ADD
   			marker.scale.x = 0.01
    			marker.scale.y = 0
    			marker.scale.z = 0
    			marker.color.a = 1.0
    			marker.color.r = 1.0
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
			if v1 == prev or v2 == prev or v1 == curr or v2 == curr:
				prev = curr
				continue
			if self._intersects_line(v1, v2, prev, curr):
				return True
			prev = curr
		return False
	

	def _intersects_line(self, p1, q1, p2, q2):
		o1 = self._orientation(p1, q1, p2)
		o2 = self._orientation(p1, q1, q2)
		o3 = self._orientation(p2, q2, p1)
		o4 = self._orientation(p2, q2, q1)

		if o1 != o2 and o3 != o4:
			return True
		if o1 == 0 and self._on_segment(p1, p2, q1):
			return True
		if o2 == 0 and self._on_segment(p1, q2, q1):
			return True
		if o3 == 0 and self._on_segment(p2, p1, q2):
			return True
		if o4 == 0 and self._on_segment(p2, q1, q2):
			return True

		return False

	def _on_segment(self, p, q, r):
		if (q.x <= max(p.x, r.x) and q.x >= min(p.x, r.x) and \
		    q.y <= max(p.y, r.y) and q.y >= min(p.y, r.y)):
			return True
		return False

	def _orientation(self, p, q, r):
		val = ((q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y))
		if val == 0:
			return 0 # colinear
		elif val > 0:
			return 1 # clockwise
		return 2 # counter clockwise
 
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
