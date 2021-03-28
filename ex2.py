import matplotlib.pyplot as plt
import csv

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def calculateDistance(self, Point):
        return (self.x-Point.x)*(self.x-Point.x)+(self.y*Point.y)*(self.y*Point.y)

    def deseneaza(self): 
        plt.plot(self.x, self.y,  color='blue', marker='o', markerfacecolor='blue', markersize=12)
    
    def __eq__(self, p):
        if abs(p.x-self.x)<1 and abs(p.y-self.y)<1.5:
            return True
        return False

    def __str__(self):
        return str(self.x) + " " + str(self.y)

# calculam pozitia unui punct fata de o dreapta determinate de 2 puncte 
def pozPunct(A,B,C):
    return (C.y-A.y)*(B.x-A.x) > (B.y-A.y)*(C.x-A.x)

    
# calculeaza cooeficientii ecuatiei dreptei determinata de p1 si p2
def lineEq(p1,p2):
    A = (p1.y - p2.y)
    B = (p2.x - p1.x)
    C = (p1.x*p2.y - p2.x*p1.y)
    return A, B, -C

# intersectia a doua drepte 
def lineIntersection(L1, L2):
    D  = L1[0] * L2[1] - L1[1] * L2[0]
    Dx = L1[2] * L2[1] - L1[1] * L2[2]
    Dy = L1[0] * L2[2] - L1[2] * L2[0]
    if D != 0:
        x = Dx / D
        y = Dy / D
        return Point(x,y)
    else:
        return False


class Edge:
    def __init__(self, p1, p2):
        self.p1 = p1
        self.p2 = p2
        self.distance = p1.calculateDistance(p2)

    def intersect(self, edg):
        if pozPunct(self.p1,edg.p1,edg.p2) != pozPunct(self.p2,edg.p1,edg.p2) and pozPunct(edg.p1,edg.p2,self.p1) != pozPunct(edg.p1,edg.p2,self.p2):
            return True
        return False

    def intersectAnyEdge(self, edgeList):
        for edge in edgeList:
            if self.intersect(edge):
                return edge
        return False

    def deseneaza(self):
        plt.plot([self.p1.x, self.p2.x], [self.p1.y, self.p2.y],  color='blue', marker='o', markerfacecolor='red', markersize=12)

    def __str__(self):
        return str(self.p1) + " " + str(self.p2)
    
class Figure:
    def __init__(self, pointList, edgeList):
        self.pointList = pointList
        self.edgeList = edgeList

    def addPoint(self, Point):
        self.pointList.append(Point)
    def addEdge(self, Edge):
        self.edgeList.append(Edge)
    
    def deseneaza(self):
        for edge in self.edgeList:
            plt.plot([edge.p1.x, edge.p2.x], [edge.p1.y, edge.p2.y],  color='red', marker='o', markerfacecolor='red', markersize=12)

# creaza edges care nu se intersecteaza cu alte edges 
def createFreeEdges(point, pointList, edgeList):
    listaEdge = []
    for p in pointList:
        newEdge = Edge(point, p)
        segmentIntersection =  newEdge.intersectAnyEdge(edgeList)
        if segmentIntersection!=False:
            intersecitonPoint = lineIntersection(lineEq(newEdge.p1,newEdge.p2), lineEq(segmentIntersection.p1, segmentIntersection.p2))
            print(str(intersecitonPoint) + " " + str(p))
            if intersecitonPoint == p:
                listaEdge.append(newEdge)
    return listaEdge



def main():
    listaPuncte = [Point(1,0),Point(0,0), Point(0,1), Point(1,1), Point(3,3), Point(3,4)]
    listaDistante = [Edge(listaPuncte[0],listaPuncte[1]), Edge(listaPuncte[1],listaPuncte[2]),Edge(listaPuncte[2],listaPuncte[3]),Edge(listaPuncte[3],listaPuncte[0]), Edge(listaPuncte[4],listaPuncte[5])]

    points = []
    with open('Points.csv', 'r') as file:
            reader = csv.reader(file)
            next(reader)
            for row in reader:
                    x = int(row[0])
                    y = int(row[1])
                    points.append(Point(x,y))
    edges = []
    with open('Edges.csv', 'r') as file:
            reader = csv.reader(file)
            next(reader)
            for row in reader:
                    p1 = int(row[0])
                    p2 = int(row[1])
                    edges.append(Edge(points[p1-1],points[p2-1]))

    fig2 = Figure(points,edges)

    startPoint = Point(94,199)
    endPoint = Point(471,23)

    fig1 = Figure(listaPuncte,listaDistante)


    # p1=Point(1,1)
    # p2=Point(2,3)
    # p3=Point(0,0)
    # p4=Point(0,4)
    # plt.plot(p1.x, p1.y,  color='blue', marker='o', markerfacecolor='blue', markersize=12)
    # plt.plot(p2.x, p2.y,  color='blue', marker='o', markerfacecolor='blue', markersize=12)
    # plt.plot(p3.x, p3.y,  color='red', marker='o', markerfacecolor='red', markersize=12)
    # plt.plot(p4.x, p4.y,  color='red', marker='o', markerfacecolor='red', markersize=12)
    # e1 = Edge(p1,p2)
    # e2 = Edge(p3,p4)
    # print(e1.intersect(e2))
    
    # e1.deseneaza()
    # e2.deseneaza()

    

    startPoint.deseneaza()
    endPoint.deseneaza()
    fig2.deseneaza()
    #plt.plot(5, 10,  color='blue', marker='o', markerfacecolor='blue', markersize=12)


    listPosibilitati = createFreeEdges(startPoint, points, edges)
    for edge in listPosibilitati:
        print(str(edge))
        edge.deseneaza()


    plt.show()
main()
