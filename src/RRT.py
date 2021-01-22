import math
import random
import numpy as np

np.seterr(divide="ignore", invalid="ignore")


class RRTPlanner:
    """
    Class for rrt planning
    """

    class Node:
        def __init__(self, point, parent):
            self.point = point
            self.parent = parent

    def __init__(
        self, boundary, blocks, expand_dis=3, goal_sample_rate=5, max_iter=500
    ):
        self.boundary = boundary
        self.blocks = blocks
        self.expand_dis = expand_dis
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter

    def plan(self, start, goal):
        """
        Main rrt algorithm
        """
        start_node = self.Node(start, None)
        goal_node = self.Node(goal, None)

        self.node_list = [start_node]
        path = None

        while path is None:
            rand_node = self.get_random_node(goal_node)
            nearest_node = self.get_nearest_node(rand_node)
            new_node = self.steer(nearest_node, rand_node)

            # if the new point and the path between two nodes are valid
            # append to node list
            if self.check_collision(new_node.point) == False:
                if self.is_valid(nearest_node.point, new_node.point):
                    self.node_list.append(new_node)

            # try to steer toward the goal node if we are close enough
            if self.get_distance(self.node_list[-1], goal_node) <= self.expand_dis:
                if self.is_valid(self.node_list[-1].point, goal_node.point):
                    goal_node.parent = self.node_list[-1]
                    path = self.extract_path(start_node, goal_node)
        return path

    def get_random_node(self, goal_node):
        """
        Random sampling
        """
        if random.randint(0, 100) > self.goal_sample_rate:
            x_rand = random.uniform(self.boundary[0, 0], self.boundary[0, 3])
            y_rand = random.uniform(self.boundary[0, 1], self.boundary[0, 4])
            z_rand = random.uniform(self.boundary[0, 2], self.boundary[0, 5])
            rand_node = self.Node([x_rand, y_rand, z_rand], None)
        else:  # goal point sampling
            rand_node = self.Node(goal_node.point, None)
        return rand_node

    def get_nearest_node(self, rand_node):
        """
        Get the nearest node to the random sampled node
        """
        dist_list = [self.get_distance(node, rand_node) for node in self.node_list]
        minIdx = dist_list.index(min(dist_list))
        return self.node_list[minIdx]

    def steer(self, start, goal):
        """
        Return a point in the direction of the goal, that is distance away from start
        """
        dist = self.get_distance(start, goal)
        v = np.array(start.point) - np.array(goal.point)
        u = v / dist
        new_node = self.Node(start.point + u * self.expand_dis, start)
        return new_node

    def get_distance(self, start, goal):
        """
        Get the L2-norm between start node and the goal node
        """
        distance = np.sqrt(((start.point - goal.point) ** 2).sum())
        return distance

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