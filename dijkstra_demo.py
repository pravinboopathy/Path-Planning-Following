import wa_simulator as wa
import matplotlib.pyplot as plt
import numpy as np
import math



def main():

    # Load track data points from a csv file
    wa.set_wa_data_directory('data/')
    filename = wa.get_wa_data_file("paths/sample_medium_loop.csv")
    waypoints = wa.load_waypoints_from_csv(filename, delimiter=",")

    # Create the path
    path = wa.WASplinePath(waypoints, num_points=1000, is_closed=True)
    # Create the track with a constant width
    track = wa.create_constant_width_track(path, width=10)
    
    path = getDjikstraPath(track)

    # Plot track boundaries with our new path
    plt.axis('equal')
    path.plot("red", show=False)
    track.left.plot("black", show=False)
    track.right.plot("black")
    

class Node:
    """
    Represents a Node with x, y values and visited status

    Attributes
        x, y (float): the x and y values of the node's location
        cd: cummulative distance
        edges (list): list of edges connected to the node
        previous: pointer to previous node
    """

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.cd = 100000#arbitrarily high number
        self.edges = []
        self.previous = None
    def set_previous(self, previous):
        self.previous = previous
    

class Edge:
    """
    Represents an Edge between two nodes

    Attributes
        a, b (Node): nodes connected by the edge
        cost (float): cost of traveling from node a to b (distance)
    """

    def __init__(self, a, b):
        self.a = a
        self.b = b
        # COST FUNCTION (currently distance)
        self.cost = math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2)

class Segment:
    """
    Attributes
        nodes: list of nodes housed on segment
    """
    def __init__(self, nodes):
        self.nodes = nodes
    
def createSegmentation(width, left, right, center, index, size):
    """
    generate Node objects from left to right boundaries (CENTERLINE BASED)

    Args:
        width (int): width of segmentations
        left, right, center (Path): left, right, and center paths of track
        index (int): index of point on center line
        size (int): number of Node objects on segment

    Returns:
        list : a list of nodes along a segmentation
    """
    nodes = []

    # center.dy / center.dx
    slope = center.get_points(der=1)[index][1] / center.get_points(der=1)[index][0]

    # s = slope, w = track width
    # x = length of x componenet, y = length of y component
    # x^2 + y^2 = w^2
    # y = (-x/s)

    # using these we solve for the length of x and y:
    # x = (+-)sqrt(w^2 / (1 + (1 / s^2)))
    # y = (+-)sqrt(w^2 - (w^2 / (1 + (1 / s^2))))

    lenX = math.sqrt((width**2) / (1 + (1 / (slope**2))))
    lenY = math.sqrt(width**2 - ((width**2) / (1 + (1 / (slope**2)))))

    #left = outer border, right = inner
    for i in range(size):
        if left.calc_closest_point(center.get_points()[index]).x > right.calc_closest_point(center.get_points()[index]).x:
            nodeX = right.calc_closest_point(center.get_points()[index]).x + (lenX * i/(size-1))
        else:
            nodeX = right.calc_closest_point(center.get_points()[index]).x - (lenX * i/(size-1))

        if left.calc_closest_point(center.get_points()[index]).y > right.calc_closest_point(center.get_points()[index]).y:
            nodeY = right.calc_closest_point(center.get_points()[index]).y + (lenY * i/(size-1))
        else:
            nodeY = right.calc_closest_point(center.get_points()[index]).y - (lenY * i/(size-1))

        nodes.append(Node(nodeX, nodeY))

    return nodes

def createWaypoints(start_seg, num_segs):#creates waypoints tracking previous nodes
    nodes = start_seg.nodes
    dist_dic = {node: node.cd for node in nodes}
    min_dist_node = min(dist_dic, key = lambda node: dist_dic[node])#returns node with min cummulative distance

    path_nodes = []
    
    curr = min_dist_node
    for i in range(num_segs+1):
        path_nodes.append(curr)
        curr = curr.previous#last iteration, previous will be None but won't append
    waypoints = [[node.x, node.y, 0] for node in path_nodes]
    return waypoints
    

def getDjikstraPath(track):
    
    num_partition = 50
    segments = []

    width = 7
    track = wa.create_constant_width_track(track.center, width=width)
    size = 5

    for i in range(num_partition+1):
        if i == num_partition:
            segmentation = createSegmentation(width, track.left, track.right, track.center, int(0*(len(track.center.get_points())/num_partition)), size)
        else:
            segmentation = createSegmentation(width, track.left, track.right, track.center, int(i*(len(track.center.get_points())/num_partition)), size)
        
        segments.append(Segment(segmentation))
        nodes = segments[i].nodes
        
        prev_seg_nodes = []

        if i == 0:#first iteration so cd must 0, not 10,0000 and must not check previous segments
            for node in nodes:
                node.cd = 0
            print("cd's have been initialized")
        else:#ensures no null pointer 
            prev_seg_nodes = segments[i-1].nodes
            for a in nodes:
                a.edges = [Edge(a, b) for b in prev_seg_nodes]#creates edges
                edges_cost = {edge: edge.cost for edge in a.edges}
                
                for i in range(len(edges_cost)):#greedy approach using min edge
                    min_edge = min(edges_cost, key = lambda edge: edges_cost[edge])#computes edge with minimum cost
                    min_edge_key = min(edges_cost, key=edges_cost.get)#computes key of min_edge
                    if(a.cd >  min_edge.b.cd + min_edge.cost):
                        a.cd = min_edge.b.cd + min_edge.cost
                        a.previous = min_edge.b#sets previous to other node in min_edge
                    edges_cost.pop(min_edge_key)
    
    #creates path
    start_seg = segments[num_partition]
    waypoints = createWaypoints(start_seg, num_partition)
    path = wa.WASplinePath(waypoints, num_points=1000)
    return path
    


# Will call the main function when 'python path_demo.py' is run
if __name__ == "__main__":
    main()
