import csv

class Distanta:
        def __init__(self,location1, location2, distance):
                self.location1 = location1
                self.location2 = location2
                self.distance = distance
        def __str__(self):
                return self.location1 +" " + self.location2 + " " + str(self.distance)



def main():
        print("Hello!")
        listaDistante = []
        with open('distante.csv', 'r') as file:
                reader = csv.reader(file)
                next(reader)
                for row in reader:
                        begin = row[0]
                        end = row[1]
                        dist = row[2]
                        listaDistante.append(Distanta(location1=begin, location2 = end, distance= dist))
        for dist in listaDistante:
                print(str(dist))
main()


