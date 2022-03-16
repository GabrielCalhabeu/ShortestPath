import time

#Formato utilizado para medir distancia, predecessor e a posição
#Desta maneira ao inves de ter 2 listas separadas usei uma so lista.

class Dijkstra_Format:
    def __init__(self, pos, dist):
        self.pos = pos
        self.dist = dist
        self.pred = None
        self.isPoped = False

    def __eq__(self, other):
        if not isinstance(other, type(self)): return NotImplemented
        return self.pos == other.pos

    def __repr__(self):
        return f'\nPos: {self.pos} | Distancia: {self.dist} | Predecessor: {self.pred}'

    def __str__(self):
        return f'{self.pos}, {self.dist}, {self.pred}'

    @staticmethod
    def minDist(list):
        min = float('inf')
        pos = None
        for aresta in list:
            if(aresta.isPoped == False and aresta.dist <= min):
                min = aresta.dist
                pos = aresta.pos
        return pos

    @staticmethod
    def pathFinder(minList, origem, destino):
        shortestPath = []
        shortestPath.append(destino)

        next = minList[destino].pred

        shortestPath.append(next)
        while (next != origem):
            try:
                next = minList[next].pred
            except TypeError:
                return None
            shortestPath.append(next)
        shortestPath.reverse()
        return shortestPath

class Grafo:

    #Definição de grafo
    def __init__(self, filename):

        #Le e processa o arquivo txt
        with open(filename, 'r') as f:
            nums = [list(map(int,line.split(" "))) for line in f.read().split('\n')]

        #Cria a lista e os vertices

        self.vertices = (nums[0][0])
        self.grafo = [[] for i in range(self.vertices)]
        self.peso = [[] for i in range(self.vertices)]

        #Vertice        0       1       2
        #exepmlo = [ [1, 2] , [2, 0], [0, 1] ]
        #peso =    [ [2, 1] , [4, 6], [1, 3] ]

        #Adiciona as arestas
        for triplet in nums[1:]:
            self.adiciona_aresta(triplet[0],triplet[1],triplet[2])

    #Adicionar arestas e peso ao grafo
    def adiciona_aresta(self, u, v, p):
        self.grafo[u].append(v)
        self.peso[u].append(p)

    def mostra_lista(self):
        for i in range(self.vertices):
            print(f'{i} ->', end= ' ')
            for j, obj in enumerate(self.grafo[i]):
                peso = self.peso[i][j]
                print(f'({obj}, {peso})', end=' ')
            print('')

#-----------------------------------Dijkstra------------------------------------
    def dijkstra(self, origem):

        distList = []

        for i in range(self.vertices):
            distList.append(Dijkstra_Format(i, float('inf')))

        distList[origem] = Dijkstra_Format(origem, 0)

        j = 0
        while j != self.vertices:
            u = Dijkstra_Format.minDist(distList)
            distList[u].isPoped = True
            j = j+1
            for i, v in enumerate(self.grafo[u]):
                if distList[v].dist > (distList[u].dist + self.peso[u][i]):
                    distList[v].dist = distList[u].dist + self.peso[u][i]
                    distList[v].pred = u

        return distList

#-----------------------------------BellmanFord---------------------------------
    def BellmanFord(self, origem):

            distList = []
            for i in range(self.vertices):
                distList.append(Dijkstra_Format(i, float('inf')))

            distList[origem] = Dijkstra_Format(origem, 0)

            j = 0
            while j != self.vertices:
                j = j+1
                trocou = False
                for u, k in enumerate(self.grafo):
                    for i, v in enumerate(k):
                        if distList[v].dist > (distList[u].dist + self.peso[u][i]):
                            distList[v].dist = distList[u].dist + self.peso[u][i]
                            distList[v].pred = u
                            trocou = True
                if trocou == False:
                    break;

            return distList

#---------------------------------Floyd-Warshall

    def Solution(self, pred, distList, origem):
        j = 0
        for i in range(self.vertices):
            solution = [Dijkstra_Format(i, distList[origem][i]) for i in range(self.vertices)]
        for j in range(self.vertices):
            solution[j].pred = pred[origem][j]
        return solution

    def FloydWarshall(self, origem):

        distList = [[float('inf')]*self.vertices for i in range(self.vertices)]
        pred = [[None]*self.vertices for i in range(self.vertices)]

        #Vertice      0       1         2
        #exepmlo = [ [1, 2] , [2, 0], [0, 1] ]
        #peso =    [ [2, 1] , [4, 6], [1, 3] ]

        for i in range(self.vertices):
            for j in range(self.vertices):
                if(i == j):                                                     #Exepmlo i = 0, j = 1
                    distList[i][j] = 0                                          #==---0--1--2--
                else:                                                           # 0|  0  2  i
                    try:                                                        # 1|  i  0  i
                        distList[i][j] = self.peso[i][self.grafo[i].index(j)]   # 2|  i  i  0
                        pred[i][j] = i
                    except(ValueError, IndexError):
                        pass

        for k in range(self.vertices):
            for l in range(self.vertices):
                for m in range(self.vertices):
                    if(distList[l][m] > distList[l][k] + distList[k][m]):
                        distList[l][m] = distList[l][k] + distList[k][m]
                        pred[l][m] = pred[k][m]


        return self.Solution(pred, distList, origem)

#---------------------------------- MAIN ---------------------------------------

class Interface:
    @staticmethod
    def interface():
        arquivo = input("Informe o arquivo: ")

        print("1 - Dijkstra")
        print("2 - Bellman-Ford")
        print("3 - Floyd-Warshall")
        algoritmo = input("Algoritmos: ")
        origem = int(input("Origem: "))
        destino = int(input("Destino: "))
        print("Processando...")


        f = Grafo(arquivo)
        start_time = time.time()

        if algoritmo == '1':
            min = f.dijkstra(origem)

        elif algoritmo == '2':
            min = f.BellmanFord(origem)

        elif algoritmo == '3':
            min = f.FloydWarshall(origem)

        end_time = time.time() - start_time

        shortestPath = Dijkstra_Format.pathFinder(min, origem, destino)

        print("Caminho:", shortestPath)
        print("Custo:", min[destino].dist)
        print("Tempo: %.5s s" % end_time)

Interface.interface()
