## Lab 3
**Rachel Wu (rww2115)**  
**David Lee (jl4397)**

*COMSW4733 Computational Aspects of Robotics*  
*Peter Allen*

#### Dependencies
```
numpy==1.15.2
scipy==1.1.0
```

#### To Run
This assumes that you have installed ROS Indigo on Ubuntu 14.04 and have cloned the [`rbx1` repository](https://github.com/pirobot/rbx1).

First, `cd` into this repository
```
$ cd <path_to_this_repo>
```

To run the script, open two terminal windows. In one window, run
```
$ roslaunch vgraph launch.launch
```

In another terminal window, run
```
$ python lab3.py
```

#### Methods
**Part 2**  

**Part 3**  
`connect_lines()`: Iterates through all pairs of vertices, draws line segments for ones that don't intersect an object, and adds the pair to self.traversable_edges  
`_intersects_any_object(v1, v2, vertex_points)`: Given a list of lists representing vertexes of a shape, figure out if line segment v1-v2 intersects any of the shapes  
`_intersects_object(v1, v2, object_vertices)`: Given the vertexes of an object, determine if line segment v1-v2 intersects any of the edges of the object  
`_intersects_line(v1, v2, v3, v4)`: Return True if line segment v1-v2 intersects segment v3-v4. Uses orientation as described here (http://www.dcs.gla.ac.uk/~pat/52233/slides/Geometry1x1.pdf)  
`_on_segment(v1, v2, point)`: Returns True if point is on line segment v1-v2  
`_orientation(v1, v2, v3)`: Returns if the direction of lines connecting v1-v2, v2-v3, and v3-v1 are collinear, clockwise, or counter-clockwise  
`_distance(v1, v2)`: Returns the distance between v1 and v2, which are 3-D tuples  

**Part 4**  
`get_shortest_path()`: Converts self.traversable_edges to an adjacency matrix and returns the shortest path of the matrix  
`_draw_path(path)`: Draws a shortest path, represented as a list of 3-D tuples  
`dijkstra(graph, start, end)`: Runs Dijkstra's algorithm to find shortest path from start to end  

**Part 5**  


#### Video
https://youtu.be/tuT9Vcz1MxA
