class Dijkstra:
    def __init__(self):
        self.path = []

    def findShortestPath(self, graph, src, target):
        """
        params:
        src: id of the source node.
        target: id of the target node 
        """
        parent = []  # parent array to store shortest path tree
        dist = []  # output array, dist[i] will hold the shortest distance from src to i
        self.source = src

        # sptSet[i] will true if vertex i is included / in shortest path tree or shortest distance from src to i is finalized
        sptSet = []

        # Initialize all distances as INFINITE and stpSet[] as false
        for i in range(len(graph)):
            parent.append(-1)
            dist.append(float("inf"))
            sptSet.append(False)
        
        # Distance of source vertex from itself is always 0
        dist[src] = 0.0

        # Find shortest path for all vertices
        for i in range(len(graph)):
            # Pick the minimum distance vertex from the set of vertices not yet processed. u is always equal to src
            # in first iteration.
            u = self.minDistance(dist, sptSet)
            #  Mark the picked vertex as processed
            sptSet[u] = True
            # Update dist value of the adjacent vertices of the picked vertex.
            for v in range(len(graph)):
                if not sptSet[v] and graph[u].neighbors[v] and dist[u] + graph[u].neighbors[v] < dist[v]:
                    parent[v] = u
                    dist[v] = dist[u] + graph[u].neighbors[v]
        
        return self.getSolution(parent, dist, target)

    def minDistance(self, dist, sptSet):
        minValue = float("inf")
        min_index = 0

        for v in range(len(dist)):
            if sptSet[v] == False and dist[v] <= minValue:
                minValue = dist[v]
                min_index = v
        return min_index
    
    def getSolution(self, parent, dist, target):
        self.path.clear()
        if(dist[target] >= float("inf")):
            raise Exception("Dijkstra: No Path Found")
        self.path.append(self.source)
        self.getPath(parent, target)
        return self.path

    def getPath(self, parent, j):
        if parent[j] == -1:
            return
        self.getPath(parent, parent[j])
        self.path.append(j)
