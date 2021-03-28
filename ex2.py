import numpy as np
import matplotlib.pyplot as plt
import shapely.geometry
import descartes
import random


from math import sqrt
import csv

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def calculateDistance(self, Point):
        return int(sqrt((Point.x-self.x)**2+(Point.y-self.y)**2))

    def deseneaza(self): 
        plt.plot(self.x, self.y,  color='blue', marker='o', markerfacecolor='blue', markersize=12)

    def __hash__(self):
        return hash((self.x, self.y))

    def __eq__(self, p):
        return self.x == p.x and self.y == p.y

    def __str__(self):
        return str(self.x) + " " + str(self.y)

class Edge:
    def __init__(self, p1, p2):
        self.p1 = p1
        self.p2 = p2
        self.distance = p1.calculateDistance(p2)

    def __str__(self):
        return str(self.p1) + " " + str(self.p2)

# This class represent a graph
class AStarGraph:
    # Initialize the class
    def __init__(self, graph_dict=None, directed=True):
        self.graph_dict = graph_dict or {}
        self.directed = directed
        if not directed:
            self.make_undirected()
    # Create an undirected graph by adding symmetric edges
    def make_undirected(self):
        for a in list(self.graph_dict.keys()):
            for (b, dist) in self.graph_dict[a].items():
                self.graph_dict.setdefault(b, {})[a] = dist
    # Add a link from A and B of given distance, and also add the inverse link if the graph is undirected
    def connect(self, A, B, distance=1):
        self.graph_dict.setdefault(A, {})[B] = distance
        if not self.directed:
            self.graph_dict.setdefault(B, {})[A] = distance
    # Get neighbors or a neighbor
    def get(self, a, b=None):
        links = self.graph_dict.setdefault(a, {})
        if b is None:
            return links
        else:
            return links.get(b)
    # Return a list of nodes in the graph
    def nodes(self):
        s1 = set([k for k in self.graph_dict.keys()])
        s2 = set([k2 for v in self.graph_dict.values() for k2, v2 in v.items()])
        nodes = s1.union(s2)
        return list(nodes)
# This class represent a node
class AStarNode:
    # Initialize the class
    def __init__(self, name:str, parent:str):
        self.name = name
        self.parent = parent
        self.g = 0 # Distance to start node
        self.h = 0 # Distance to goal node
        self.f = 0 # Total cost
    # Compare nodes
    def __eq__(self, other):
        return self.name == other.name
    # Sort nodes
    def __lt__(self, other):
         return self.f < other.f
    # , node
    def __repr__(self):
        return ('({0},{1})'.format(self.name, self.f))
# A* search
def AStarSearch(graph, heuristics, start, end):
    
    # Create lists for open nodes and closed nodes
    open = []
    closed = []
    # Create a start node and an goal node
    start_node = AStarNode(start, None)
    goal_node = AStarNode(end, None)
    # Add the start node
    open.append(start_node)
    
    # Loop until the open list is empty
    while len(open) > 0:
        # Sort the open list to get the node with the lowest cost first
        open.sort()
        # Get the node with the lowest cost
        current_node = open.pop(0)
        # Add the current node to the closed list
        closed.append(current_node)
        
        # Check if we have reached the goal, return the path
        if current_node == goal_node:
            path = []
            while current_node != start_node:
                path.append(current_node.name)
                current_node = current_node.parent
            path.append(start_node.name)
            # Return reversed path
            return path[::-1]
        # Get neighbours
        neighbors = graph.get(current_node.name)
        # Loop neighbors
        for key, value in neighbors.items():
            # Create a neighbor node
            neighbor = AStarNode(key, current_node)
            # Check if the neighbor is in the closed list
            if(neighbor in closed):
                continue
            # Calculate full path cost
            neighbor.g = current_node.g + graph.get(current_node.name, neighbor.name)
            neighbor.h = heuristics.get(neighbor.name)
            neighbor.f = neighbor.g + neighbor.h
            # Check if neighbor is in open list and if it has a lower f value
            if(add_to_open(open, neighbor) == True):
                # Everything is green, add neighbor to open list
                open.append(neighbor)
    # Return None, no path is found
    return None
# Check if a neighbor should be added to open list
def add_to_open(open, neighbor):
    for node in open:
        if (neighbor == node and neighbor.f > node.f):
            return False
    return True


def reachableNodes(startPoint, pointList, shapeList, edgeList):
    nodes = []
    for p in pointList:
        line = shapely.geometry.LineString([[startPoint.x, startPoint.y], [p.x, p.y]])
        isIntersecting = False
        for kante in edgeList:
            if kante.p1 == p or kante.p2 == p or kante.p1 == startPoint or kante.p2 == startPoint:
                continue
            else:
                second_line = shapely.geometry.LineString([(kante.p1.x, kante.p1.y), (kante.p2.x, kante.p2.y)])
                if line.intersects(second_line):
                    isIntersecting = True
                for shape in shapeList:
                    if shape.contains(line):
                        isIntersecting = True
        if isIntersecting is False:
            nodes.append(p)
    return nodes
        

def intersectShape(line, shapeList):
    for shape in shapeList:
        if(line.intersects(shape)):
            return True
    return False

def isInsideShape(x,y,shapeList):
    shapelyPoint = shapely.geometry.Point(x,y)
    for shape in shapeList:
        if shape.contains(shapelyPoint):
            return True
    return False

def main():
    shapeList = []

    shapeList.append(shapely.geometry.Polygon([[110,170],[110,222],[284,222],[284,170]]))
    shapeList.append(shapely.geometry.Polygon([[308,197],[343,162],[291,121]]))
    shapeList.append(shapely.geometry.Polygon([[410,140],[374,166],[373,207],[408,226],[441,206],[440,165]]))
    shapeList.append(shapely.geometry.Polygon([[449,146],[462,44],[440,22],[413,38]]))
    shapeList.append(shapely.geometry.Polygon([[403,135],[332,135],[332,22],[403,22]]))
    shapeList.append(shapely.geometry.Polygon([[320,46],[293,17],[248,21],[249,88]]))
    shapeList.append(shapely.geometry.Polygon([[220,62],[198,147],[245,147]]))

    shapeList.append(shapely.geometry.Polygon([[154,16],[194,70],[161,141],[102,128],[94,73]]))

    edgeList = []
    # shape 1
    edgeList.append(Edge(Point(110,170),Point(110,222)))
    edgeList.append(Edge(Point(110,222),Point(284,222)))
    edgeList.append(Edge(Point(284,222),Point(284,170)))
    edgeList.append(Edge(Point(284,170),Point(110,170)))
    # shape 2
    edgeList.append(Edge(Point(308,197),Point(343,162)))
    edgeList.append(Edge(Point(343,162),Point(291,121)))
    edgeList.append(Edge(Point(291,121),Point(308,197)))
    # shape 3
    edgeList.append(Edge(Point(410,140),Point(374,166)))
    edgeList.append(Edge(Point(374,166),Point(373,207)))
    edgeList.append(Edge(Point(373,207),Point(408,226)))
    edgeList.append(Edge(Point(408,226),Point(441,206)))
    edgeList.append(Edge(Point(441,206),Point(440,165)))
    edgeList.append(Edge(Point(440,165),Point(410,140)))
    # shape 4
    edgeList.append(Edge(Point(449,146),Point(462,44)))
    edgeList.append(Edge(Point(462,44),Point(440,22)))
    edgeList.append(Edge(Point(440,22),Point(413,38)))
    edgeList.append(Edge(Point(413,38),Point(449,146)))
    # shape 5
    edgeList.append(Edge(Point(403,135),Point(332,135)))
    edgeList.append(Edge(Point(332,135),Point(332,22)))
    edgeList.append(Edge(Point(332,22),Point(403,22)))
    edgeList.append(Edge(Point(403,22),Point(403,135)))
    # shape 6
    edgeList.append(Edge(Point(320,46),Point(293,17)))
    edgeList.append(Edge(Point(293,17),Point(248,21)))
    edgeList.append(Edge(Point(248,21),Point(249,88)))
    edgeList.append(Edge(Point(249,88),Point(320,46)))
    # shape 7
    edgeList.append(Edge(Point(220,62),Point(198,147)))
    edgeList.append(Edge(Point(198,147),Point(245,147)))
    edgeList.append(Edge(Point(245,147),Point(220,62)))
    #shape 8
    edgeList.append(Edge(Point(154,16),Point(194,70)))
    edgeList.append(Edge(Point(194,70),Point(161,141)))
    edgeList.append(Edge(Point(161,141),Point(102,128)))
    edgeList.append(Edge(Point(102,128),Point(94,73)))
    edgeList.append(Edge(Point(94,73),Point(154,16)))

    points = []
    with open('Points.csv', 'r') as file:
            reader = csv.reader(file)
            next(reader)
            for row in reader:
                    x = int(row[0])
                    y = int(row[1])
                    points.append(Point(x,y))

    for nrRep in range(100):  
        print("Nr rep: ", nrRep+1)  

        
        randX= random.randint(0,400)
        randY= random.randint(0,300)

        while isInsideShape(randX, randY, shapeList):
            randX= random.randint(0,400)
            randY= random.randint(0,300)

        startPoint = Point(randX,randY)
        endPoint = Point(469,24)

        fig = plt.figure()
        ax = fig.add_subplot(111)
        for shape in shapeList:
            ax.add_patch(descartes.PolygonPatch(shape, fc='blue', alpha=0.5))


         

        dictVecini = {}
        dictVecini[startPoint]=reachableNodes(startPoint,points,shapeList, edgeList)
        dictVecini[endPoint]=reachableNodes(endPoint,points, shapeList, edgeList)
        for point in points:
            dictVecini[point]=reachableNodes(point,points, shapeList, edgeList)

        
        heueristics = {}
        for p in points:
            heueristics[p] = startPoint.calculateDistance(p)
        heueristics[startPoint] = 0
        heueristics[endPoint] = startPoint.calculateDistance(endPoint)

        aStarGraph = AStarGraph()
        for node in points:
            for vecin in dictVecini[node]:
                aStarGraph.connect(node, vecin, node.calculateDistance(vecin))
        for vecin in dictVecini[startPoint]:
            aStarGraph.connect(startPoint, vecin, startPoint.calculateDistance(vecin))
        for vecin in dictVecini[endPoint]:
            aStarGraph.connect(endPoint, vecin, endPoint.calculateDistance(vecin))
        aStarGraph.make_undirected()
        # c)
        proc = random.randint(0,100)
        if proc < 30:
            print("wrong")
            randX= random.randint(0,400)
            randY= random.randint(0,300)

            while isInsideShape(randX, randY, shapeList):
                randX= random.randint(0,400)
                randY= random.randint(0,300)
            endPoint = Point(randX,randY)

            dictVecini = {}
            dictVecini[startPoint]=reachableNodes(startPoint,points,shapeList, edgeList)
            dictVecini[endPoint]=reachableNodes(endPoint,points, shapeList, edgeList)
            for point in points:
                dictVecini[point]=reachableNodes(point,points, shapeList, edgeList)

            
            heueristics = {}
            for p in points:
                heueristics[p] = startPoint.calculateDistance(p)
            heueristics[startPoint] = 0
            heueristics[endPoint] = startPoint.calculateDistance(endPoint)

            aStarGraph = AStarGraph()
            for node in points:
                for vecin in dictVecini[node]:
                    aStarGraph.connect(node, vecin, node.calculateDistance(vecin))
            for vecin in dictVecini[startPoint]:
                aStarGraph.connect(startPoint, vecin, startPoint.calculateDistance(vecin))
            for vecin in dictVecini[endPoint]:
                aStarGraph.connect(endPoint, vecin, endPoint.calculateDistance(vecin))
            aStarGraph.make_undirected()
            
            line = shapely.geometry.LineString([[startPoint.x, startPoint.y], [endPoint.x, endPoint.y]])
            ax.plot(*np.array(line).T, color="r", linewidth=3, solid_capstyle='round')
            if intersectShape(line, shapeList):
                path = AStarSearch(aStarGraph, heueristics, startPoint, endPoint)  
               
                colors = ['#4287f5','#ff4f87','#ffac30','#85fffd','#be85ff']

                sum = 0
                for i in range(len(path)-1):
                    sum += path[i].calculateDistance(path[i+1])
                    line = shapely.geometry.LineString([[path[i].x, path[i].y], [path[i+1].x, path[i+1].y]])
                    ax.plot(*np.array(line).T, color=colors[i%3], linewidth=3, solid_capstyle='round')

                score = 1000 - sum
                print("Score:", score)  
            else:
                score = 1000 - startPoint.calculateDistance(endPoint)
                print("Score:", score)
        else:
            path = AStarSearch(aStarGraph, heueristics, startPoint, endPoint)

            colors = ['#4287f5','#ff4f87','#ffac30','#85fffd','#be85ff']

            sum = 0
            for i in range(len(path)-1):
                sum += path[i].calculateDistance(path[i+1])
                line = shapely.geometry.LineString([[path[i].x, path[i].y], [path[i+1].x, path[i+1].y]])
                ax.plot(*np.array(line).T, color=colors[i%3], linewidth=3, solid_capstyle='round')

            score = 1000 - sum
            print("Score:", score)

        ax.axis('equal')

        plt.plot(startPoint.x, startPoint.y,  color='red', marker='o', markerfacecolor='red', markersize=12)
        plt.plot(endPoint.x, endPoint.y,  color='red', marker='o', markerfacecolor='red', markersize=12)

        plt.show()
main()

"""
a) cel mai scurt drum intre 2 puncte este o dreapta; daca definim o dreapta de la un punct la un varf, Zustandsraum se transforma din inifinit in ceva finit
d) daca d este foarte mic merge direct la endNode fara sa treaca prin varf daca se intersecteaza cu un Kante antuni face A*
"""