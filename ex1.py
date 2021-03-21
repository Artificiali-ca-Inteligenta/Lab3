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



def main():
        listaDistante = []
        with open('distante.csv', 'r') as file:
                reader = csv.reader(file)
                next(reader)
                for row in reader:
                        begin = row[0]
                        end = row[1]
                        dist = row[2]
                        listaDistante.append(Distanta(location1=begin, location2 = end, distance= dist))

        G = nx.Graph()
        for dist in listaDistante:
          G.add_edge(dist.location1,dist.location2)

        H = nx.Graph()

        print("Nodes: ")
        print(G.nodes())
        print("Edges: ")
        print(G.edges())
        
        plt.axis("off")
        plt.figure(figsize = (10,10))
        nx.draw_networkx(G, node_size = 2500,font_size = 12, pos=nx.kamada_kawai_layout(G))
        plt.savefig("grap.png")
main()