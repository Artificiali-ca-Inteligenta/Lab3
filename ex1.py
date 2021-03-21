import csv
import networkx as nx
import matplotlib.pyplot as plt

class Distanta:
        def __init__(self,location1, location2, distance):
                self.location1 = location1
                self.location2 = location2
                self.distance = distance
        def __str__(self):
                return self.location1 +" " + self.location2 + " " + str(self.distance)
class Heueristic:
        def __init__(self, name, distance):
                self.name = name
                self.distance = distance
        def __str__(self):
                return self.name + " " + str(self.distance)

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

def main():
        listaDistante = []
        with open('distante.csv', 'r') as file:
                reader = csv.reader(file)
                next(reader)
                for row in reader:
                        begin = row[0]
                        end = row[1]
                        dist = int(row[2])
                        listaDistante.append(Distanta(location1=begin, location2 = end, distance= dist))
        listaHeueristic = []
        with open('heueristics.csv', 'r') as file:
                reader = csv.reader(file)
                next(reader)
                for row in reader:
                        name = row[0]
                        dist = int(row[1])
                        listaHeueristic.append(Heueristic(name=name, distance= dist))


        aStarGraph = AStarGraph()
        for elem in listaDistante:
                aStarGraph.connect(elem.location1, elem.location2 ,elem.distance)

        aStarGraph.make_undirected()

        heueristics = {}
        for heueristic in listaHeueristic:
                heueristics[heueristic.name] = heueristic.distance


        path = AStarSearch(aStarGraph, heueristics, "Bucharest", "Paris")
        print(path)


        
        labels = {}
        G = nx.Graph()
        for dist in listaDistante:
                G.add_edge(dist.location1,dist.location2)
                labels[(dist.location1,dist.location2)] = dist.distance

        color_map = []
        for node in G:
                if node in path:
                        color_map.append('#ff6b61')
                else: 
                        color_map.append('skyblue')


        
        
        plt.axis("off")
        plt.figure(figsize = (15,15))
        nx.draw_networkx(G,node_color = color_map, node_size = 2500,font_size = 12, pos=nx.kamada_kawai_layout(G))
        nx.draw_networkx_edge_labels(G, edge_labels=labels, pos=nx.kamada_kawai_layout(G), font_size= 12)
        plt.savefig("grap.png")
main()
