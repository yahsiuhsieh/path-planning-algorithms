import math
import random
import numpy as np
from RRT import RRTPlanner

np.seterr(divide="ignore", invalid="ignore")


class RRTStarPlanner(RRTPlanner):
    """
    Class for rrt star planning
    """

    class Node(RRTPlanner.Node):
        def __init__(self, point, parent):
            super().__init__(point, parent)
            self.cost = 0

    def __init__(
        self, boundary, blocks, expand_dis=3, goal_sample_rate=5, max_iter=500
    ):
        super().__init__(boundary, blocks, expand_dis, goal_sample_rate, max_iter)

    def plan(self, start, goal):
        """
        Main rrt star algorithm
        """
        start_node = self.Node(start, None)
        goal_node = self.Node(goal, None)

        self.node_list = [start_node]
        path = None

        while path is None:
            rand_node = self.get_random_node(goal_node)
            nearest_node = self.get_nearest_node(rand_node)
            new_node = self.steer(nearest_node, rand_node)
            new_node.cost = nearest_node.cost + self.get_distance(
                nearest_node, new_node
            )

            # if the new point and the path between two nodes are valid
            if self.check_collision(new_node.point) == False:
                if self.is_valid(nearest_node.point, new_node.point):
                    near_idxs = self.find_near_nodes(new_node)
                    node_with_updated_parent = self.update_parent(new_node, near_idxs)
                    if node_with_updated_parent:
                        self.rewire(node_with_updated_parent, near_idxs)
                        self.node_list.append(node_with_updated_parent)
                    else:
                        self.node_list.append(new_node)

            # try to steer toward the goal node if we are close enough
            if self.get_distance(self.node_list[-1], goal_node) <= self.expand_dis:
                if self.is_valid(self.node_list[-1].point, goal_node.point):
                    goal_node.parent = self.node_list[-1]
                    path = self.extract_path(start_node, goal_node)
        return path

    def find_near_nodes(self, center_node):
        """
        Returns list with the indices of the nodes inside the ball which centered on given node
        """
        dist_list = [self.get_distance(center_node, node) for node in self.node_list]
        near_idxs = [dist_list.index(d) for d in dist_list if d <= self.expand_dis]
        return near_idxs

    def update_parent(self, node, near_idxs):
        """
        Compute the cheapest node to the given node in the list near_idxs
        """
        if not near_idxs:
            return None

        # search smallest cost in near_idxs
        cost = []
        for idx in near_idxs:
            near_node = self.node_list[idx]
            to_node = self.steer(near_node, node)

            if self.check_collision(to_node.point) and self.is_valid(
                near_node.point, to_node.point
            ):
                cost.append(near_node.cost + self.get_distance(near_node, to_node))
            else:
                cost.append(float("inf"))  # the cost of collision

        min_cost = min(cost)
        if min_cost == float("inf"):
            return None

        min_idx = near_idxs[cost.index(min_cost)]
        new_node = self.steer(self.node_list[min_idx], node)
        new_node.cost = min_cost
        return new_node

    def rewire(self, node, near_idxs):
        """
        For each node in near_idxs, check if the cost will be lower to arrive them from the given node
        """
        for idx in near_idxs:
            near_node = self.node_list[idx]
            to_node = self.steer(node, near_node)
            to_node.cost = node.cost + self.get_distance(node, to_node)

            no_collision = self.is_valid(node.point, to_node.point)
            improved_cost = to_node.cost < near_node.cost

            if no_collision and improved_cost:
                near_node = to_node
                self.propogate_cost_to_leaves(near_node)

    def propogate_cost_to_leaves(self, parent):
        for node in self.node_list:
            if node.parent == parent:
                node.cost = parent.cost + self.get_distance(parent, node)
                self.propogate_cost_to_leaves(node)
