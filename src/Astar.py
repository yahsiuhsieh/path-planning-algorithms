import numpy as np
from heapq import heappush, heappop


class AStarPlanner:
    """
    Class for a star planning
    """

    class Node:
        def __init__(self, point, cost, parent):
            self.point = point
            self.cost = cost
            self.parent = parent

        def __lt__(self, other):
            return self.cost < other.cost

    def __init__(self, boundary, blocks):
        self.boundary = boundary
        self.blocks = blocks

    def plan(self, start, goal):
        """
        Main a star algorithm
        """
        start_node = self.Node(start, 0, None)
        goal_node = self.Node(goal, 0, None)

        # setting a priority queue for extracting nodes with least cost
        openSet = []
        heappush(openSet, (0, start_node))
        node_list = []
        visited = set()  # using set data structure for faster search

        while len(openSet):
            # get the lowest cost node
            current_node = heappop(openSet)[1]
            node_list.append(current_node)
            visited.add(tuple(current_node.point))

            # goal reach condition
            if np.sqrt(((current_node.point - goal_node.point) ** 2).sum()) < 0.5:
                goal_node.parent = current_node
                path = self.extract_path(start_node, goal_node)
                print("Reach Goal")
                return path

            # explore valid neighbors
            neighbors = self.explore_neighbors(current_node)
            g_score = (
                current_node.cost + 0.5
            )  # cost to get to next is 0.5 in all directions
            for nb in neighbors:
                # if the neighbor is not visited before
                # add to the priority queue
                if set(nb) not in visited:
                    new_node = self.Node(nb, g_score, current_node)
                    f_score = g_score + 3 * self.heuristic(new_node, goal_node)
                    heappush(openSet, (f_score, new_node))
                    continue

                # if visited before
                # check if there is a better way to reach that node
                tmp = self.search(node_list, nb)
                if tmp.cost > g_score:
                    tmp.cost = g_score
                    f_score = g_score + 3 * self.heuristic(tmp, goal_node)
                    heappush(openSet, (f_score, tmp))
                    tmp.parent = current_node

    def explore_neighbors(self, node):
        """
        Explore the surrounding neighbors of the current node
        """
        neighbors = []
        [dX, dY, dZ] = np.meshgrid([-1, 0, 1], [-1, 0, 1], [-1, 0, 1])
        dR = np.vstack((dX.flatten(), dY.flatten(), dZ.flatten()))
        dR = np.delete(
            dR,
            [0, 1, 2, 3, 5, 6, 7, 8, 9, 11, 13, 15, 17, 18, 19, 20, 21, 23, 24, 25, 26],
            axis=1,
        )
        dR = dR / 2

        for k in range(len(dR[0])):
            nb = node.point + dR[:, k]
            nb = np.round(nb, 2)
            # Check if this direction is valid
            if self.is_valid(node.point, nb) == False:
                continue
            else:
                neighbors.append(nb)
        return neighbors

    def is_valid(self, from_node, to_node):
        """
        Check if the path between two nodes is valid and collision free
        """
        # if point out of boundary
        if (
            to_node[0] <= self.boundary[0, 0]
            or to_node[0] >= self.boundary[0, 3]
            or to_node[1] <= self.boundary[0, 1]
            or to_node[1] >= self.boundary[0, 4]
            or to_node[2] <= self.boundary[0, 2]
            or to_node[2] >= self.boundary[0, 5]
            or self.check_collision(to_node)
        ):
            return False

        # if line go through AABB
        for i in range(3):
            direction = to_node - from_node
            tmp = from_node + (i + 1) * direction / 3
            if self.check_collision(tmp):
                return False
        return True

    def check_collision(self, point):
        """
        Check if the point is in collision
        """
        for k in range(self.blocks.shape[0]):
            if (
                point[0] >= self.blocks[k, 0] - 0.1
                and point[0] <= self.blocks[k, 3] + 0.1
                and point[1] >= self.blocks[k, 1] - 0.1
                and point[1] <= self.blocks[k, 4] + 0.1
                and point[2] >= self.blocks[k, 2] - 0.1
                and point[2] <= self.blocks[k, 5] + 0.1
            ):
                return True
        return False

    def extract_path(self, start, goal):
        """
        Extract path from start node to goal node
        """
        path = []
        node = goal
        while node.parent is not None:
            path.append(node.point.tolist())
            node = node.parent
        path.append(start.point.tolist())
        path.reverse()
        return np.array(path)

    def search(self, nodes, point):
        """
        Search if the point is in the set nodes, return the node if found
        """
        for node in nodes:
            if (node.point == point).all():
                return node
        return None

    def heuristic(self, node, goal):
        """
        Calculate the heuristic between current node and the goal node
        """
        z_heu = np.abs(node.point[2] - goal.point[2])
        return self.get_distance(node, goal)  # + 30*z_heu

    def get_distance(self, start, goal):
        """
        Get the L1-norm between start node and the goal node
        """
        # distance = np.sqrt(((start.point - goal.point)**2).sum())
        distance = np.abs((start.point - goal.point)).sum()
        return np.round(distance, 2)
