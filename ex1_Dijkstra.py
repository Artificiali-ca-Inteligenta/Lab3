import csv
import sys
import time
import heapq
import networkx as nx
import matplotlib.pyplot as plt

class Vertex:
    def __init__(self, node):
        self.id = node
        self.adjacent = {}
        self.distance = sys.maxsize
        self.visited = False
        self.previous = None

    def add_neighbor(self, neighbor, weight=0):
        self.adjacent[neighbor] = weight

    def get_connections(self):
        return self.adjacent.keys()  

    def get_id(self):
        return self.id

    def get_weight(self, neighbor):
        return self.adjacent[neighbor]

    def set_distance(self, dist):
        self.distance = dist

    def get_distance(self):
        return self.distance

    def set_previous(self, prev):
        self.previous = prev

    def set_visited(self):
        self.visited = True

    def __str__(self):
        return str(self.id) + ' adjacent: ' + str([x.id for x in self.adjacent])

    def __lt__(self, other):
        return self.id < other.id

class Graph:
    def __init__(self):
        self.vert_dict = {}
        self.num_vertices = 0

    def __iter__(self):
        return iter(self.vert_dict.values())

    def add_vertex(self, node):
        self.num_vertices = self.num_vertices + 1
        new_vertex = Vertex(node)
        self.vert_dict[node] = new_vertex
        return new_vertex

    def get_vertex(self, n):
        if n in self.vert_dict:
            return self.vert_dict[n]
        else:
            return None

    def add_edge(self, frm, to, cost = 0):
        if frm not in self.vert_dict:
            self.add_vertex(frm)
        if to not in self.vert_dict:
            self.add_vertex(to)

        self.vert_dict[frm].add_neighbor(self.vert_dict[to], cost)
        self.vert_dict[to].add_neighbor(self.vert_dict[frm], cost)

    def get_vertices(self):
        return self.vert_dict.keys()

    def set_previous(self, current):
        self.previous = current

    def get_previous(self, current):
        return self.previous

def shortest(v, path):
    if v.previous:
        path.append(v.previous.get_id())
        shortest(v.previous, path)
    return

def dijkstra(aGraph, start, target):
    print("Dijkstra's shortest path")
    start.set_distance(0)

    unvisited_queue = [(v.get_distance(),v) for v in aGraph]
    heapq.heapify(unvisited_queue)

    while len(unvisited_queue):
        uv = heapq.heappop(unvisited_queue)
        current = uv[1]
        current.set_visited()

        for next in current.adjacent:
            if next.visited:
                continue
            new_dist = current.get_distance() + current.get_weight(next)
            
            if new_dist < next.get_distance():
                next.set_distance(new_dist)
                next.set_previous(current)
                #print('updated : current = '+current.get_id()+' next = '+next.get_id()+' new_dist = '+str(next.get_distance()))
            #else:
                #print('not updated : current = '+current.get_id()+' next = '+next.get_id()+' new_dist = '+str(next.get_distance()))

        while len(unvisited_queue):
            heapq.heappop(unvisited_queue)
        unvisited_queue = [(v.get_distance(),v) for v in aGraph if not v.visited]
        heapq.heapify(unvisited_queue)

class Distanta:
        def __init__(self,location1, location2, distance):
                self.location1 = location1
                self.location2 = location2
                self.distance = distance
        def __str__(self):
                return self.location1 +" " + self.location2 + " " + str(self.distance)


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
        
        labels={}
        G = nx.Graph()
        gDijkstra = Graph()
        for dist in listaDistante:
            G.add_edge(dist.location1,dist.location2)
        for node in G.nodes():
            gDijkstra.add_vertex(node)
        for dist in listaDistante:
            gDijkstra.add_edge(dist.location1,dist.location2,dist.distance)
            labels[(dist.location1,dist.location2)] = dist.distance

        dijkstra(gDijkstra, gDijkstra.get_vertex('Bucharest'), gDijkstra.get_vertex('Paris'))

        target = gDijkstra.get_vertex('Paris')
        path = [target.get_id()]
        shortest(target, path)

        color_map = []
        for node in G:
                if node in path:
                        color_map.append('#ff6b61')
                else: 
                        color_map.append('#73ff8f')

        print('The shortest path :' + str(path[::-1]))

        plt.axis("off")
        plt.figure(figsize = (15,15))
        nx.draw_networkx(G,node_color = color_map, node_size = 2500,font_size = 12, pos=nx.kamada_kawai_layout(G))
        nx.draw_networkx_edge_labels(G, edge_labels=labels, pos=nx.kamada_kawai_layout(G), font_size= 12)
        plt.savefig("Dijkstra.png")
        
main()