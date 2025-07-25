import heapq

class MinRootGraph:
    def __init__(self):
        self.waypoint = set()
        self.edges = {}
        self.distances = {}
        
    def add_waypoint(self, value):
        self.waypoint.add(value)
        self.edges[value] = []
        
    def add_edge(self, current_waypoint, goal_waypoint, cost):
        cost = cost.value
        self.edges[current_waypoint].append((goal_waypoint, cost))
        self.edges[goal_waypoint].append((current_waypoint, cost))
        self.distances[(current_waypoint, goal_waypoint)] = cost
        self.distances[(goal_waypoint, current_waypoint)] = cost
        
    def dijkstra(self, initial):
        visited = {initial: 0}
        path = {}
        pq = [(0, initial)]

        while pq:
            current_distance, current_node = heapq.heappop(pq)

            if current_distance > visited[current_node]:
                continue

            for neighbor, weight in self.edges[current_node]:
                # print("aaaaa", type(weight))
                distance = current_distance + weight
                if neighbor not in visited or distance < visited[neighbor]:
                    visited[neighbor] = distance
                    path[neighbor] = current_node
                    heapq.heappush(pq, (distance, neighbor))

        return visited, path