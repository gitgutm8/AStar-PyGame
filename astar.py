import priorityqueue as pq
from collections import defaultdict
import math


class Vector(tuple):

    def __add__(self, other): return Vector(v + w for v, w in zip(self, other))
    def __sub__(self, other): return Vector(v - w for v, w in zip(self, other))
    def __mul__(self, scalar): return Vector(v * scalar for v in self)
    def __truediv__(self, scalar): return Vector(v / scalar for v in self)
    def __floordiv__(self, scalar): return Vector(v // scalar for v in self)
    def __ceil__(self): return Vector(math.ceil(v) for v in self)
    def __neg__(self): return -1 * self
    def length(self): return sum(v**2 for v in self) ** 0.5


class AStar:

    def __init__(self, adjacent_nodes_function, heuristic_function, weights_function):
        """

        :param adjacent_nodes_function:
            A function that takes one node as input and returns all nodes
            reachable from it

        :param heuristic_function:
            A function that takes two nodes as input, one of them being the
            goal node, and computes a value that helps at identifying which
            node to pick next.

        :param weights_function:
            A function that takes two (adjacent) nodes and calculates the
            cost of going from one to the other.
        """
        self.adj = adjacent_nodes_function
        self.heuristic = heuristic_function
        self.weight = weights_function
        self.cache = {}

    def _compute_path(self, source, goal):
        """
        Calculates the path connecting two given nodes with the least
        possible weight using the A*-algorithm.

        :param source:
            The node the path is supposed to start on.

        :param goal:
            The node where the path is supposed to end on.

        :return:
            Dictionary with parent for each node where the parent is the node
            which provides the shortest path to a given node.
        """
        parent = {source: None}
        shortest = defaultdict(lambda: float('inf'))
        shortest[source] = 0

        node_pq = pq.PriorityQueue((source, 0))

        while node_pq:
            node = node_pq.pop_task()
            if node == goal:
                return self._restore_path(parent, source, goal)

            for adj_node in self.adj(node):
                cost = shortest[node] + self.weight(node, adj_node)
                if shortest[adj_node] > cost:
                    shortest[adj_node] = cost
                    parent[adj_node] = node
                    node_pq.add_task(adj_node, cost + self.heuristic(adj_node, goal))
        return []

    @staticmethod
    def _restore_path(parent_dic, source, goal):
        """
        Computes the order of the nodes from `goal` to `source`, knowing
        the predecessor to each node.

        :param parent_dic:
            Dictionary mapping each node to its predecessor, where the
            predecessor is the node guaranteeing the shortest path to a
            given node.

        :param source:the start node
        :param goal: the end node

        :return:
            a list of all nodes needed to reach `goal` from `source` with
            the least possible overall weight.
        """
        path = []
        node = goal
        while node is not source:
            path.append(node)
            node = parent_dic[node]
        return path[::-1]

    def _register(self, source, goal):
        """
        Adds the shortest path connecting `source` and `goal` to
        the cache.

        :param source: the start node
        :param goal: the end node
        :return: None
        """
        path = self._compute_path(source, goal)
        # no path found
        if not path:
            return
        self.cache[(source, goal)] = path

    def get_shortest(self, source, goal):
        """
        Return the shortest path connecting two nodes, uses cached value
        if possible. Raises ValueError if there is no path connecting them.

        :param source: the start node
        :param goal: the end node
        :return: the shortest path connecting the nodes
        """
        if (source, goal) not in self.cache:
            self._register(source, goal)
        if (source, goal) in self.cache:
            return self.cache[(source, goal)]
        raise ValueError('no path connecting these nodes')

    def invalidate(self):
        """
        Cleans the cache, used if a change to the graph occurs.

        :return: None
        """
        self.cache = {}


class StringStar(AStar):

    def __init__(self, template, *args, **kwargs):
        self.string = template
        self.adj_dict = {}
        self._parse_template()
        super().__init__(self.adj, *args, **kwargs)

    def adj(self, node):
        for adj_node in self.adj_dict[node]:
            yield adj_node

    def _parse_template(self):
        array = [[c for c in line] for line in self.string.splitlines()]
        for y, line in enumerate(array):
            for x, tile in enumerate(line):
                if tile == '#':
                    continue
                adj_nodes = []
                for neighbour, pos in self._get_neighbours(x, y, array):
                    if neighbour == '#':
                        continue
                    adj_nodes.append(Vector(pos))
                self.adj_dict[Vector((x, y))] = adj_nodes

    def draw_path(self, source, goal):
        path = self.get_shortest(source, goal)
        array = [[c for c in line] for line in self.string.splitlines()]
        for x, y in path:
            array[y][x] = '.'
        print('\n'.join(''.join(row) for row in array))

    @staticmethod
    def _get_neighbours(x, y, array):
        neighbours = [
            (-1, -1), (-1, 0), (-1, 1),
            ( 0, -1),          ( 0, 1),
            ( 1, -1), ( 1, 0), ( 1, 1)
        ]
        max_x, max_y = max(len(line) for line in array), len(array)
        for dy, dx in neighbours:
            x2, y2 = x+dx, y+dy
            if 0 <= x2 < max_x and 0 <= y2 < max_y:
                yield array[y2][x2], (x2, y2)


if __name__ == '__main__':
    def f_adj(node):
        neighbours = [
            (-1, -1), (-1, 0), (-1, 1),
            ( 0, -1),          ( 0, 1),
            ( 1, -1), ( 1, 0), ( 1, 1)
        ]
        x, y = node
        for dy, dx in neighbours:
            x2, y2 = x+dx, y+dy
            if 0 <= x2 < 10 and 0 <= y2 < 10:
                yield Vector((x2, y2))

    def f_weight(node, neighbor):
        return round((neighbor-node).length(), 2)

    a = AStar(f_adj, f_weight, f_weight)
    #print(a.get_shortest(Vector((0, 0)), Vector((9, 4))))

    sa = StringStar("""\
            
     #####  
         #  
         #  
         #  
            
            
""", f_weight, f_weight)

    sa.draw_path(Vector((0, 6)), Vector((11, 0)))


